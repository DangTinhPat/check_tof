#!/usr/bin/env python3
"""
VL53L8CX depth viewer - serial bridge + web server.

This is the ONLY program that opens the board's serial port. Browsers never touch
the port; they receive the frames from this server over Server-Sent Events, so any
number of tabs / browsers can watch at the same time without fighting for the port.

    python3 server.py                 # auto-detect /dev/ttyACM* or /dev/ttyUSB*
    python3 server.py --port /dev/ttyACM0 --http 8000

Then open http://localhost:8000 in any browser.
Stop it (Ctrl+C) before flashing the board with idf.py.

Standard library only (no pyserial needed).
"""
import argparse, fcntl, re, glob, json, os, queue, select, struct, sys, termios, threading, time
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
        self.logs = []

    def _push(self, event, data):
        msg = f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n".encode()
        for q in list(self.clients):
            try:
                q.put_nowait(msg)
            except queue.Full:          # slow client: drop this message, keep going
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

    def add_frame(self, n, d):
        with self.lock:
            self.frame = {"n": n, "d": d}
            self._push("frame", self.frame)

    def add_log(self, line):
        with self.lock:
            self.logs = (self.logs + [line])[-200:]
            self._push("log", line)

    def subscribe(self):
        q = queue.Queue(maxsize=200)
        with self.lock:
            self.clients.add(q)
            # bring the new client up to date immediately
            q.put_nowait(f"event: status\ndata: {json.dumps(self.status, ensure_ascii=False)}\n\n".encode())
            for line in self.logs[-50:]:
                q.put_nowait(f"event: log\ndata: {json.dumps(line, ensure_ascii=False)}\n\n".encode())
            if self.frame:
                q.put_nowait(f"event: frame\ndata: {json.dumps(self.frame)}\n\n".encode())
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.clients.discard(q)


HUB = Hub()


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
        if line.startswith("F,"):
            p = line.split(",")
            if len(p) == NZ + 2:
                try:
                    n = int(p[1]); d = [int(x) for x in p[2:]]
                except ValueError:
                    return False
                HUB.add_frame(n, d)
                return True
            return False
        if re.fullmatch(r"[-0-9,]+", line):   # tail of a frame line cut in half when the port was opened
            return False
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
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif self.path == "/events":
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
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/reset":
            WORKER.reset_req.set()
            self._send(200, b'{"ok":true}', "application/json")
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
