#!/usr/bin/env python3
"""
VL53L8CX depth viewer - serial bridge + web server.

This is the ONLY program that opens the board's serial port. Browsers never touch
the port; they receive the frames from this server over Server-Sent Events, so any
number of tabs / browsers can watch at the same time without fighting for the port.

    python3 server.py                 # auto-detect /dev/ttyACM* or /dev/ttyUSB*
    python3 server.py --port /dev/ttyACM0 --http 8000

Data: the firmware sends raw frames ("F2,<frame>,<t_ms>,<64 dist>,<64 status>,<64 sigma>");
this server only forwards them. All filtering happens in the browser (filters.js).
Raw lines can be recorded to gui/recordings/*.csv (button in the GUI or POST /api/record).

Then open http://localhost:8000 in any browser.
Stop it (Ctrl+C) before flashing the board with idf.py.

Standard library only (no pyserial needed).
"""
import argparse, fcntl, re, glob, json, os, queue, select, socket, struct, sys, termios, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

HERE = os.path.dirname(os.path.abspath(__file__))
NZ = 64
BAUD = 921600                # must match CONFIG_ESP_CONSOLE_UART_BAUDRATE in examples/sdkconfig.defaults
STALE_AFTER = 3.0          # s without frames while the port is open -> "stale"


# --------------------------------------------------------------------------- hub
class Hub:
    """Holds the latest state and fans events out to every connected browser."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = set()
        self.status = {"state": "starting", "msg": "Đang khởi động…"}
        self.frame = None
        self.cfg = None
        self.logs = []

    def _push(self, event, data):
        msg = f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False, separators=(',', ':'))}\n\n".encode()
        for q in list(self.clients):
            try:
                q.put_nowait(msg)
            except queue.Full:
                # slow client: for real-time display the NEWEST data matters, so drop the
                # oldest queued message to make room instead of dropping this one
                try:
                    q.get_nowait()
                    q.put_nowait(msg)
                except (queue.Empty, queue.Full):
                    pass

    def set_status(self, state, msg, **extra):
        st = {"state": state, "msg": msg, **extra}
        with self.lock:
            if st == self.status:
                return
            self.status = st
            self._push("status", st)
        print(f"[{time.strftime('%H:%M:%S')}] {state}: {msg}", flush=True)
        try:
            with open(os.path.join(HERE, "board.log"), "a", encoding="utf-8") as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  [server] {state}: {msg}\n")
        except OSError:
            pass

    def add_frame(self, frame):
        with self.lock:
            self.frame = frame
            self._push("frame", frame)

    def set_cfg(self, cfg):
        with self.lock:
            self.cfg = cfg
            self._push("cfg", cfg)

    def add_log(self, line):
        with self.lock:
            self.logs = (self.logs + [line])[-200:]
            self._push("log", line)

    def subscribe(self):
        q = queue.Queue(maxsize=60)        # ~4 s of frames; older ones are dropped first
        with self.lock:
            self.clients.add(q)
            # bring the new client up to date immediately
            q.put_nowait(f"event: status\ndata: {json.dumps(self.status, ensure_ascii=False)}\n\n".encode())
            for line in self.logs[-50:]:
                q.put_nowait(f"event: log\ndata: {json.dumps(line, ensure_ascii=False)}\n\n".encode())
            if self.cfg:
                q.put_nowait(f"event: cfg\ndata: {json.dumps(self.cfg)}\n\n".encode())
            if self.frame:
                q.put_nowait(f"event: frame\ndata: {json.dumps(self.frame, separators=(',', ':'))}\n\n".encode())
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.clients.discard(q)


HUB = Hub()


# ---------------------------------------------------------------------- recorder
class Recorder(threading.Thread):
    """Writes raw lines to gui/recordings/<time>.csv from its own thread.

    The serial reader only does a non-blocking queue put; if the disk ever stalls,
    lines are counted as dropped instead of delaying the live stream."""

    DIR = os.path.join(HERE, "recordings")

    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue(maxsize=5000)
        self.on = False
        self.path = None
        self.lines = 0
        self.dropped = 0
        self.started = None

    def info(self):
        size = 0
        if self.path:
            try:
                size = os.path.getsize(self.path)
            except OSError:
                pass
        return {"on": self.on, "file": os.path.relpath(self.path, HERE) if self.path else None,
                "lines": self.lines, "dropped": self.dropped, "bytes": size,
                "seconds": round(time.time() - self.started, 1) if self.on and self.started else 0}

    def start_recording(self):
        if self.on:
            return
        os.makedirs(self.DIR, exist_ok=True)
        self.path = os.path.join(self.DIR, time.strftime("%Y%m%d-%H%M%S") + ".csv")
        self.lines = self.dropped = 0
        self.started = time.time()
        self.q.put(("open", self.path))
        self.on = True

    def stop_recording(self):
        if self.on:
            self.on = False
            self.q.put(("close",))

    def put(self, line):
        if not self.on:
            return
        try:
            self.q.put_nowait(("line", time.time(), line))
        except queue.Full:
            self.dropped += 1

    def run(self):
        f, last_flush = None, 0.0
        while True:
            item = self.q.get()
            try:
                if item[0] == "open":
                    if f:
                        f.close()
                    f = open(item[1], "w", encoding="utf-8", buffering=1 << 16)
                    f.write("# host_time_s,raw_line  (raw_line: F2,<frame>,<t_ms>,<64 dist mm>,"
                            "<64 status>,<64 sigma mm> or a board log line)\n")
                elif item[0] == "close":
                    if f:
                        f.close()
                        f = None
                elif item[0] == "line" and f:
                    f.write(f"{item[1]:.3f},{item[2]}\n")
                    self.lines += 1
                    if time.time() - last_flush > 1.0:
                        f.flush()
                        last_flush = time.time()
            except OSError as e:
                print(f"[recorder] {e}", flush=True)
                self.on = False


REC = Recorder()


# ------------------------------------------------------------------------ serial
def find_port(wanted):
    if wanted:
        return wanted if os.path.exists(wanted) else None
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    return None


def port_owners(path):
    """Names/PIDs of other processes that have `path` open (helps explain 'busy')."""
    owners = []
    try:
        real = os.path.realpath(path)
        for pid in os.listdir("/proc"):
            if not pid.isdigit() or int(pid) == os.getpid():
                continue
            fd_dir = f"/proc/{pid}/fd"
            try:
                for fd in os.listdir(fd_dir):
                    if os.path.realpath(os.path.join(fd_dir, fd)) == real:
                        with open(f"/proc/{pid}/comm") as f:
                            owners.append(f"{f.read().strip()} (PID {pid})")
                        break
            except (PermissionError, FileNotFoundError, ProcessLookupError):
                pass
    except Exception:
        pass
    return owners


def open_serial(path):
    # Plain open asserts DTR+RTS together, which on ESP32 auto-reset circuits is the
    # "run" state, so opening the port does NOT reset the board.
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)        # nobody else may open it while we read
        a = termios.tcgetattr(fd)
        a[0] = 0                                  # iflag: raw
        a[1] = 0                                  # oflag: raw
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL   # no HUPCL: don't drop DTR/RTS on close
        a[3] = 0                                  # lflag: raw
        speed = getattr(termios, f"B{BAUD}")
        a[4] = a[5] = speed
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
        termios.tcflush(fd, termios.TCIFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def _modem(fd, set_bits=0, clear_bits=0):
    if set_bits:
        fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", set_bits))
    if clear_bits:
        fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", clear_bits))


class SerialWorker(threading.Thread):
    def __init__(self, wanted_port):
        super().__init__(daemon=True)
        self.wanted = wanted_port
        self.fd = None
        self.reset_req = threading.Event()

    def handle_line(self, line):
        if line.startswith("F2,"):
            p = line.split(",")
            if len(p) != 3 + 3 * NZ:
                return False                      # truncated line (e.g. port opened mid-line)
            try:
                v = [int(x) for x in p[1:]]
            except ValueError:
                return False
            HUB.add_frame({"n": v[0], "t": v[1], "d": v[2:2 + NZ], "s": v[2 + NZ:2 + 2 * NZ],
                           "g": v[2 + 2 * NZ:2 + 3 * NZ]})
            REC.put(line)
            return True
        if line.startswith("F,"):                 # old firmware: distances only, 0 = invalid
            p = line.split(",")
            if len(p) == NZ + 2:
                try:
                    n = int(p[1]); d = [int(x) for x in p[2:]]
                except ValueError:
                    return False
                HUB.add_frame({"n": n, "t": None, "d": d, "s": [5 if x > 0 else 255 for x in d],
                               "g": [0] * NZ, "legacy": True})
                REC.put(line)
                return True
            return False
        if re.fullmatch(r"[-0-9,]+", line):   # tail of a frame line cut in half when the port was opened
            return False
        REC.put(line)
        m = re.search(r"CFG zones=(\d+) hz=(\d+) sharpener=(\d+) order=(\w+) i2c_hz=(\d+)", line)
        if m:
            HUB.set_cfg({"zones": int(m[1]), "hz": int(m[2]), "sharpener": int(m[3]),
                         "order": m[4], "i2c_hz": int(m[5])})
        HUB.add_log(line)
        try:                                   # keep a persistent, timestamped copy for diagnosing intermittent faults
            with open(os.path.join(HERE, "board.log"), "a", encoding="utf-8") as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {line}\n")
        except OSError:
            pass
        # one failed attempt right after a reset is normal (sensor was mid-ranging);
        # only report when the firmware keeps failing
        m = re.search(r"Sensor start failed \(attempt (\d+)\)", line)
        if m and int(m.group(1)) >= 3:
            HUB.set_status("sensor", f"Board chạy nhưng không khởi động được cảm biến (lần {m.group(1)}) — "
                                     "kiểm tra dây SDA/SCL/VIN/LPn")
        return False

    def hard_reset(self):
        # same sequence as esptool "hard reset": EN low via RTS, then release
        _modem(self.fd, clear_bits=termios.TIOCM_DTR)
        _modem(self.fd, set_bits=termios.TIOCM_RTS)
        time.sleep(0.12)
        _modem(self.fd, clear_bits=termios.TIOCM_RTS)

    def run(self):
        last_note = None
        while True:
            path = find_port(self.wanted)
            if not path:
                HUB.set_status("noport", "Không thấy board — hãy cắm cáp USB")
                time.sleep(0.5)
                continue
            try:
                self.fd = open_serial(path)
            except OSError as e:
                owners = port_owners(path)
                if owners:
                    HUB.set_status("busy", f"Cổng {path} đang bị chiếm bởi: {', '.join(owners)} — hãy đóng chương trình đó",
                                   owners=owners)
                else:
                    HUB.set_status("busy", f"Không mở được {path}: {e.strerror}")
                time.sleep(1.0)
                continue

            HUB.set_status("waiting", f"Đã mở {path} — chờ dữ liệu từ cảm biến…", port=path)
            buf = b""
            last_frame = time.time()
            try:
                while True:
                    if self.reset_req.is_set():
                        self.reset_req.clear()
                        HUB.add_log("[server] reset board")
                        HUB.set_status("waiting", "Đang reset board…", port=path)
                        self.hard_reset()
                        termios.tcflush(self.fd, termios.TCIFLUSH)   # drop frames from before the reset
                        buf = b""
                        last_frame = time.time()
                    r, _, _ = select.select([self.fd], [], [], 0.25)
                    if r:
                        chunk = os.read(self.fd, 4096)
                        if not chunk:                       # hang-up: device went away
                            raise OSError("thiết bị đã ngắt")
                        buf += chunk
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            line = raw.decode("utf-8", "replace").strip()
                            if line and self.handle_line(line):
                                last_frame = time.time()
                                HUB.set_status("live", "Đang nhận dữ liệu", port=path)
                        if len(buf) > 65536:
                            buf = b""
                    if time.time() - last_frame > STALE_AFTER and HUB.status["state"] in ("live", "waiting"):
                        HUB.set_status("stale", "Cổng mở nhưng không có frame — board đang khởi động lại cảm biến?",
                                       port=path)
                    if not os.path.exists(path):
                        raise OSError("thiết bị đã bị rút")
            except OSError as e:
                HUB.set_status("lost", f"Mất kết nối USB ({e.strerror or e}) — đang chờ board cắm lại…")
            finally:
                try:
                    os.close(self.fd)
                except OSError:
                    pass
                self.fd = None
            time.sleep(0.3)


# -------------------------------------------------------------------------- http
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        static = {"/": ("index.html", "text/html; charset=utf-8"),
                  "/index.html": ("index.html", "text/html; charset=utf-8"),
                  "/filters.js": ("filters.js", "application/javascript; charset=utf-8")}
        if self.path in static:
            name, ctype = static[self.path]
            with open(os.path.join(HERE, name), "rb") as f:
                self._send(200, f.read(), ctype)
        elif self.path == "/events":
            try:                                   # send each small SSE message immediately
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            q = HUB.subscribe()
            try:
                self.wfile.write(b"retry: 1000\n\n"); self.wfile.flush()
                while True:
                    try:
                        msg = q.get(timeout=10)
                    except queue.Empty:
                        msg = b": ping\n\n"           # keep-alive, also detects closed tabs
                    self.wfile.write(msg); self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                HUB.unsubscribe(q)
        elif self.path == "/api/status":
            self._send(200, json.dumps(HUB.status, ensure_ascii=False).encode(), "application/json")
        elif self.path == "/api/record":
            self._send(200, json.dumps(REC.info()).encode(), "application/json")
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/reset":
            WORKER.reset_req.set()
            self._send(200, b'{"ok":true}', "application/json")
        elif self.path == "/api/record":
            n = int(self.headers.get("Content-Length") or 0)
            try:
                want = bool(json.loads(self.rfile.read(n) or b"{}").get("on"))
            except ValueError:
                want = False
            REC.start_recording() if want else REC.stop_recording()
            self._send(200, json.dumps(REC.info()).encode(), "application/json")
        else:
            self._send(404, b"not found", "text/plain")


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=BAUD, help=f"serial baud rate (default {BAUD}, must match the firmware)")
    ap.add_argument("--http", type=int, default=8000, help="HTTP port (default 8000)")
    ap.add_argument("--host", default="127.0.0.1", help="bind address (default 127.0.0.1)")
    args = ap.parse_args()

    if not hasattr(termios, f"B{args.baud}"):
        sys.exit(f"Baud rate {args.baud} không được hỗ trợ (thử 115200, 460800, 921600, 1000000, 2000000)")
    BAUD = args.baud
    REC.start()
    WORKER = SerialWorker(args.port)
    WORKER.start()
    try:
        srv = Server((args.host, args.http), Handler)
    except OSError as e:
        sys.exit(f"Không mở được cổng HTTP {args.http}: {e.strerror}. Có server khác đang chạy? "
                 f"Tắt nó bằng: pkill -f 'http.server {args.http}'  hoặc  pkill -f server.py")
    print(f"Mở trình duyệt: http://localhost:{args.http}   (Ctrl+C để dừng)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nĐã dừng.")
