# Check ToF — VL53L8CX trên ESP32-C6

Đọc cảm biến khoảng cách đa vùng **VL53L8CX** (lưới 8×8) bằng **ESP32-C6** qua I²C, rồi hiển thị
bản đồ độ sâu và khối 3D trên trình duyệt theo thời gian thực.

```
 VL53L8CX ──I²C──► ESP32-C6 ──UART/USB──► gui/server.py ──HTTP──► Trình duyệt (bản đồ 2D + 3D)
                  (examples/)             (giữ cổng serial,       (gui/index.html
                  gửi dữ liệu THÔ          ghi file thô)           + gui/filters.js = TOÀN BỘ bộ lọc)
```

**Nguyên tắc:** ESP32 không lọc gì — nó gửi nguyên khoảng cách, `target_status` và sigma của cả 64 ô như
cảm biến trả về. Mọi quyết định "ô nào tin được" nằm trong trình duyệt và chỉnh được ngay trên giao diện,
không cần nạp lại firmware.

## Mục lục
1. [Cấu trúc thư mục](#1-cấu-trúc-thư-mục)
2. [Thông số cảm biến](#2-thông-số-cảm-biến)
3. [Nối dây](#3-nối-dây)
4. [Yêu cầu](#4-yêu-cầu)
5. [Build và nạp firmware](#5-build-và-nạp-firmware)
6. [Mở giao diện web](#6-mở-giao-diện-web)
7. [Quy trình làm việc hằng ngày](#7-quy-trình-làm-việc-hằng-ngày)
8. [Định dạng dữ liệu](#8-định-dạng-dữ-liệu)
9. [Cấu hình firmware](#9-cấu-hình-firmware)
10. [Xử lý sự cố](#10-xử-lý-sự-cố)

---

## 1. Cấu trúc thư mục

```
check_tof/
├── README.md
├── VL53L8CX-Library/      Thư viện ESP-IDF của bên thứ ba (rjrp44/vl53l8cx, ULD v2.0.0 của ST)
├── examples/              Firmware của dự án này (project ESP-IDF "check_tof")
│   ├── CMakeLists.txt         trỏ tới thư viện cục bộ ../VL53L8CX-Library
│   ├── sdkconfig.defaults     target esp32c6, stack 8192, UART 921600, timeout I²C, tắt output không dùng
│   └── main/main.c            khởi tạo cảm biến, đo 8×8 @ 15 Hz, gửi mỗi frame thô thành một dòng
└── gui/
    ├── server.py              cầu nối serial → HTTP, ghi dữ liệu thô (chỉ dùng thư viện chuẩn của Python)
    ├── filters.js             chuỗi bộ lọc (hàm thuần, chạy được cả trong Node để kiểm thử)
    ├── index.html             giao diện web (không cần thư viện ngoài)
    ├── recordings/            file ghi dữ liệu thô (*.csv, tạo khi bấm "Ghi dữ liệu thô")
    └── board.log              log của board kèm giờ (để chẩn đoán sự cố chập chờn)
```

---

## 2. Thông số cảm biến

> Các thông số ghi **(code)** đã đối chiếu với thư viện trong repo. Các thông số còn lại lấy từ
> datasheet của ST (UM3109 / DS13754); hãy kiểm tra lại với tài liệu gốc khi thiết kế phần cứng.

**Nguyên lý.** ToF trực tiếp: laser VCSEL 940 nm (hồng ngoại) phát xung, mảng SPAD đếm photon phản xạ,
firmware trong chip tạo histogram theo thời gian bay rồi suy ra khoảng cách cho từng vùng.

| Hạng mục | Giá trị |
|---|---|
| Độ phân giải | 4×4 (16 vùng) hoặc 8×8 (64 vùng) **(code)** |
| Góc nhìn (FoV) | khoảng 45° × 45°, vuông |
| Tầm đo | khoảng 2 cm – 4 m (giảm mạnh khi có ánh sáng mặt trời; dưới ~2 cm số liệu không đáng tin) |
| Tần số đo tối đa | 4×4: khoảng 60 Hz · 8×8: khoảng 15 Hz |
| Chế độ đo | liên tục (`CONTINUOUS`) hoặc tự động (`AUTONOMOUS`, đặt được thời gian tích phân) **(code)** |
| Số vật thể mỗi vùng | tối đa 4, thứ tự `CLOSEST` hoặc `STRONGEST` **(code)** — dự án dùng 1 vật, `CLOSEST` |
| Giao tiếp | I²C tới 1 MHz **(code)**, hoặc SPI; chân `SPI_I2C_N` chọn chế độ |
| Địa chỉ I²C mặc định | `0x52` (8-bit) = `0x29` (7-bit) **(code)** |
| Chế độ nguồn | `SLEEP`, `WAKEUP`, `DEEP_SLEEP` **(code)** |
| Firmware của chip | **không nằm trong flash**: host nạp khoảng 84 KB qua I²C mỗi lần cấp nguồn (bước `init` mất vài giây) |
| Chân điều khiển | `LPn` (bật/tắt, mức thấp = tắt I²C), `INT` (ngắt khi có frame), `I2C_RST` |
| Plugin | motion indicator, detection thresholds, xtalk calibration **(code)** |

**Dữ liệu mỗi vùng:** khoảng cách (mm), độ lệch chuẩn ước lượng (`range_sigma`), độ phản xạ (%),
tín hiệu và ánh sáng nền trên mỗi SPAD, số SPAD bật, số vật phát hiện, và `target_status`.

**`target_status` thường gặp:** `5` = đo hợp lệ · `9` = hợp lệ với xung lớn (có thể là vật gộp) ·
các giá trị khác = không tin cậy. Dự án chỉ giữ 5 và 9.

**Độ chính xác và hạn chế**
- Độ nhiễu khi đứng yên đo được trong dự án này: khoảng **1.4 mm** mỗi vùng (vật phẳng, 10–50 cm).
- Vật phản xạ thấp (đen, vải), vật trong suốt, kính, gương và ánh sáng mặt trời làm đo sai hoặc mất vùng.
- Vùng ở mép vật thường cho giá trị lai giữa vật và nền; **sharpener** giúp giảm hiện tượng này.
- 8×8 chỉ là 64 điểm đo: ảnh độ sâu luôn thô, không như camera độ sâu.

---

## 3. Nối dây

| ESP32-C6 | VL53L8CX | Ghi chú |
|---|---|---|
| 3V3 | VIN | nguồn |
| 3V3 | LPn | mức cao = cảm biến hoạt động |
| GND | GND | |
| GND | SPI_I2C_N | mức thấp = chế độ I²C |
| GPIO6 | SDA/MOSI | |
| GPIO7 | SCL/CLK | |

- Firmware bật pull-up nội của ESP32 (yếu, ~45 kΩ) và chạy I²C ở **400 kHz**. Nếu board breakout của
  bạn chưa có sẵn pull-up, hãy thêm điện trở **2.2–4.7 kΩ** từ SDA và SCL lên 3V3.
- `LPn` nối cứng vào 3V3 nên **không có reset phần cứng** cho cảm biến; firmware tự khôi phục bằng phần mềm.
- Pin I²C đổi trong `examples/main/main.c` (`PIN_SDA`, `PIN_SCL`). ESP32-C6 chỉ có một cổng I²C chính
  (`I2C_NUM_0`).

---

## 4. Yêu cầu

- **ESP-IDF v5.5.x** (đã build bằng v5.5.6; thư viện yêu cầu từ v5.4). Cài tại `~/esp/esp-idf`.
- **Python 3.8+** để chạy `gui/server.py` (không cần cài thêm gói nào).
- Một trình duyệt bất kỳ (Chrome, Edge, Firefox…).
- Người dùng nằm trong nhóm `dialout` để truy cập `/dev/ttyACM0`:
  ```bash
  groups | grep dialout || sudo usermod -aG dialout $USER   # rồi đăng xuất / đăng nhập lại
  ```
- Board ESP32-C6 có chip cầu nối USB–UART (CH343, `1a86:55d3`) → xuất hiện là `/dev/ttyACM0`.

---

## 5. Build và nạp firmware

```bash
# 1. Nạp môi trường ESP-IDF (mỗi terminal mới)
. ~/esp/esp-idf/export.sh

# 2. Vào thư mục firmware
cd ~/check_tof/examples

# 3. Build
idf.py build

# 4. Nạp (dừng gui/server.py trước, xem bên dưới)
idf.py -p /dev/ttyACM0 flash
```

Lần đầu trên một máy mới, chạy `idf.py set-target esp32c6` trước bước 3.

**Xem log thay vì mở web:**
```bash
idf.py -p /dev/ttyACM0 flash monitor      # thoát bằng Ctrl+]
```

> **Mỗi lúc chỉ một chương trình được dùng cổng `/dev/ttyACM0`.** Hãy dừng `server.py`
> (`Ctrl+C` hoặc `pkill -f "python3 server.py"`) và đóng mọi serial monitor (VS Code, `idf.py monitor`,
> `screen`) trước khi nạp.

> `sdkconfig.defaults` chỉ có tác dụng khi chưa có file `sdkconfig`. Sau khi sửa `sdkconfig.defaults`,
> xoá `examples/sdkconfig` rồi build lại.

---

## 6. Mở giao diện web

```bash
cd ~/check_tof/gui
python3 server.py
```

Rồi mở **http://localhost:8000** trong trình duyệt. Giữ terminal chạy server; nhấn `Ctrl+C` để dừng.

Tuỳ chọn: `python3 server.py --port /dev/ttyACM0 --baud 921600 --http 8000 --host 127.0.0.1`
(`--baud` phải khớp `CONFIG_ESP_CONSOLE_UART_BAUDRATE` của firmware).

**Cách hoạt động.** `server.py` là chương trình **duy nhất** mở cổng serial. Trình duyệt chỉ nhận dữ liệu
từ server qua HTTP (Server-Sent Events), nên mở nhiều tab hoặc nhiều trình duyệt cùng lúc không giành
cổng với nhau. Server tự kết nối lại khi rút/cắm cáp, và báo rõ nếu cổng đang bị chương trình khác giữ.

### Giao diện

- **Bản đồ độ sâu 2D:** lưới 8×8, màu sáng = gần, tối = xa; hiện số mm trên từng ô; **viền trắng** ở mép vật
  (nơi hai ô cạnh nhau chênh lệch độ sâu lớn); ô gạch chéo = không đo được. Rê chuột lên ô để xem giá trị.
  Có chế độ "Mượt + đổ bóng" và 3 bảng màu.
- **Độ nổi 3D:** mỗi ô là một cột, **cột càng cao = càng gần**. Kéo chuột để xoay, cuộn để zoom, có 3 góc nhìn
  sẵn (Nghiêng / Nhìn thấp / Từ trên xuống).
- **Chụp nền:** chụp lúc cảnh trống, sau đó chỉ vật mới xuất hiện được tô màu, nền chuyển xám. Rê chuột
  hiện vật nhô ra bao nhiêu mm so với nền.
- **Bộ lọc** (xem mục bên dưới) và **Tuỳ chỉnh** hiển thị: viền mép, dải màu tự động hoặc cố định,
  lật ngang / lật dọc / đổi X↔Y. Mọi tuỳ chọn được nhớ trong trình duyệt.
- **Ghi dữ liệu thô:** nút trên thanh trên cùng; ghi mọi frame thô và log của board vào
  `gui/recordings/<giờ>.csv` (khoảng 29 MB mỗi giờ). Việc ghi chạy trong luồng riêng nên không làm chậm hình.
- **Dòng cấu hình:** cạnh nút ghi, hiện cấu hình thật đọc lại từ cảm biến (độ phân giải, Hz, sharpener,
  thứ tự vật, tốc độ I²C).
- **Thống kê:** FPS, số ô có giá trị, gần/xa nhất, chu kỳ frame đo bằng đồng hồ ESP32, số frame bị lỡ.
- **Nút Reset board:** khởi động lại ESP32 qua chân RTS. **Tạm dừng:** đóng băng hình.
- **Thanh trạng thái (góc phải):** cho biết đang nhận dữ liệu, mất kết nối USB, cổng bận (kèm tên chương
  trình đang giữ cổng), hay board chạy nhưng không khởi động được cảm biến.

### Bộ lọc (chạy trong trình duyệt, file `gui/filters.js`)

Chuỗi xử lý, theo thứ tự; mỗi bước bật/tắt và chỉnh được trên giao diện:

| # | Bước | Việc làm | Mặc định |
|---|---|---|---|
| 1 | Status | chỉ giữ ô có `target_status` được tích chọn; mỗi nút hiện số ô có status đó ở frame hiện tại | 5, 9 |
| 2 | Sigma | bỏ ô có độ bất định ước lượng lớn | ≤ 15 mm |
| 3 | Median theo thời gian | diệt giá trị nhảy vọt đơn lẻ (thêm 1 frame trễ khi khoảng cách đổi) | 3 frame |
| 4 | Giữ giá trị | ô mất tạm thời vẫn hiện giá trị cũ vài frame | 3 frame |
| 5 | Làm mượt | trung bình trượt; thay đổi lớn hơn ngưỡng đi thẳng, không trễ | 0.4, ngưỡng 100 mm |
| 6 | Điểm lơ lửng ở mép vật | bỏ ô nằm giữa một vùng gần và một vùng xa | tắt |

- **Hiển thị: Thô** cho xem dữ liệu không lọc gì, để so sánh.
- Ô không có giá trị hiện **mã lý do ở góc** (số status, `σ` = sigma, `mép`); ô đang giữ giá trị cũ có tam giác
  trắng ở góc. Rê chuột hiện giá trị thô, status kèm nghĩa, sigma và lý do bị loại.
- Đo trên 448 frame thật: bộ lọc mặc định giảm nhiễu giữa hai frame (p95) từ **34 mm xuống 6.9 mm** và vẫn giữ
  **96%** số ô có giá trị. Mỗi frame tốn khoảng 30 µs.

**Kiểm tra hướng ảnh:** đưa tay vào phía trên-bên trái cảm biến (nhìn từ phía sau cảm biến). Nếu ô sáng lên
sai góc thì chỉnh ở mục "Hướng ảnh". Mặc định bật "Lật ngang"; hãy xác nhận trên phần cứng của bạn.

---

## 7. Quy trình làm việc hằng ngày

| Việc | Lệnh |
|---|---|
| Xem giao diện | `cd ~/check_tof/gui && python3 server.py` → mở http://localhost:8000 |
| Sửa firmware rồi nạp | `Ctrl+C` server → `. ~/esp/esp-idf/export.sh` → `cd ~/check_tof/examples` → `idf.py -p /dev/ttyACM0 flash` → chạy lại `server.py` |
| Chỉ xem log | `idf.py -p /dev/ttyACM0 monitor` (đóng server trước) |
| Dừng server nền | `pkill -f "python3 server.py"` |
| Xem ai đang giữ cổng | `fuser -v /dev/ttyACM0` |

---

## 8. Định dạng dữ liệu

Firmware gửi mỗi frame thành **một dòng thô** trên UART (**921600 baud**), ghi ra bằng một lệnh duy nhất:

```
F2,<frame>,<t_ms>,<d0..d63>,<s0..s63>,<g0..g63>
```

| Trường | Ý nghĩa |
|---|---|
| `frame` | bộ đếm của cảm biến (0–255). Lưu ý: nó **tăng 4 mỗi frame**, nên không dùng để đếm frame lỡ |
| `t_ms` | thời gian chạy của ESP32 (ms) lúc đọc frame; giao diện dùng nó để đo chu kỳ và frame bị lỡ |
| `d` | khoảng cách (mm) như cảm biến báo, kể cả ô không hợp lệ |
| `s` | `target_status` (5 = hợp lệ, 9 = hợp lệ xung lớn, 255 = không có vật, …) |
| `g` | sigma (mm), độ bất định ước lượng |

- Các mảng theo **từng hàng** (hàng 0 trước). Khoảng 650 byte mỗi dòng, 15 dòng/giây (khoảng 11% băng thông UART).
- Khi khởi động, firmware in một dòng log `CFG zones=… hz=… sharpener=… order=… i2c_hz=…` với cấu hình thật.
- Mọi dòng khác là log của ESP-IDF; `server.py` chuyển chúng vào mục "Log từ board" và ghi vào `gui/board.log`.
- `server.py` vẫn đọc được định dạng cũ `F,<frame>,<64 khoảng cách>` của firmware trước đây.

`server.py` cung cấp: `GET /events` (luồng SSE: `frame`, `status`, `log`, `cfg`), `GET /api/status`,
`POST /api/reset`, `GET`/`POST /api/record` (bật/tắt ghi: `{"on": true}`).

**Realtime:** chu kỳ đo được 67–68 ms ở phía ESP32; độ giật khi tới máy tính khoảng 1.6 ms (độ lệch chuẩn).
Tab trình duyệt nào xử lý chậm sẽ bị bỏ frame **cũ** trước, nên luôn hiện dữ liệu mới nhất.

---

## 9. Cấu hình firmware

Các hằng số đầu `examples/main/main.c`:

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `PIN_SDA` / `PIN_SCL` | 6 / 7 | chân I²C |
| `I2C_SPEED_HZ` | 400000 | tốc độ I²C |
| `RANGING_HZ` | 15 | tần số đo (8×8 tối đa 15) |
| `FRAME_TIMEOUT_US` | 1 s | không có frame trong khoảng này → khởi động lại cảm biến |
| `MAX_I2C_ERRORS` | 10 | số lỗi I²C liên tiếp trước khi khởi động lại cảm biến |

Muốn đổi sang 4×4 ở tần số cao hơn: đổi `VL53L8CX_RESOLUTION_8X8` thành `_4X4` và `RANGING_HZ` (tối đa
khoảng 60), đồng thời sửa `GRID`, hằng `NZ`/`N` trong `gui/index.html`, `N` khi tạo pipeline trong
`gui/filters.js` và `NZ` trong `gui/server.py` (hiện cố định 64 giá trị).

**Cảm biến chạy theo mặc định của ST**, chỉ đặt độ phân giải, tần số và thứ tự vật `CLOSEST`. Sharpener để
mặc định (đọc ra là 1%). Firmware không lọc ô nào.

**Output được đọc từ cảm biến** (đặt trong `sdkconfig.defaults`): khoảng cách, status, sigma và số vật phát hiện
(dùng để báo status 255 khi không có vật). Ambient, số SPAD, tín hiệu, độ phản xạ và motion bị tắt để giảm
lưu lượng I²C (khoảng 460 byte mỗi frame thay vì 1450 byte).

**Độ bền của firmware:** không bao giờ bỏ cuộc. Nếu khởi tạo cảm biến lỗi thì thử lại mãi; nếu đang đo mà
ngừng có frame hoặc lỗi I²C liên tục thì tự khởi động lại cảm biến.

**Timeout I²C:** `sdkconfig.defaults` bật `CONFIG_VL53L8CX_I2C_TIMEOUT` (3000 ms) để một bus bị kẹt trả về
lỗi thay vì treo mãi. Giá trị phải lớn hơn thời gian truyền dài nhất (khối firmware 32 KB, khoảng 0.75 s ở 400 kHz).

**Tuỳ chọn của thư viện** (`idf.py menuconfig` → `Component config → Vl53l8cx`): số vật mỗi vùng (1–4),
tắt từng loại output để giảm RAM và lưu lượng I²C, mức reset, timeout I²C.

---

## 10. Xử lý sự cố

| Hiện tượng | Nguyên nhân thường gặp và cách xử lý |
|---|---|
| Không có `/dev/ttyACM0` | Kiểm tra cáp USB và đầu cắm; board ESP32-C6 thường có hai cổng USB — dùng cổng UART (chip CH343). Xem `lsusb \| grep 1a86` và `journalctl -k -f \| grep -i usb`. |
| `Permission denied` khi vừa cắm | Quyền được áp sau khoảng 1 giây, thử lại. Nếu kéo dài: thêm user vào nhóm `dialout`. |
| Web báo "cổng đang bị chiếm bởi …" / `port is busy` khi flash | Có chương trình khác giữ cổng (monitor của VS Code, `idf.py monitor`, server cũ). Chạy `fuser -v /dev/ttyACM0` rồi đóng chương trình đó. |
| Web: "Không kết nối được tới server" | Chưa chạy `python3 server.py`, hoặc mở file `index.html` trực tiếp. Phải mở qua http://localhost:8000. |
| Web hiện dữ liệu cũ / giao diện cũ | Tải lại cứng: `Ctrl+Shift+R`. |
| Log: `VL53L8CX not responding (status=3)` | I²C không tới được cảm biến. Kiểm tra dây SDA/SCL/VIN/GND/LPn, thêm pull-up ngoài 2.2–4.7 kΩ, hạ `I2C_SPEED_HZ` xuống 100000. Thất bại 1 lần ngay sau khi reset là bình thường (cảm biến đang đo dở); firmware tự thử lại. |
| Khoảng cách chỉ vài mm, nhiều ô trống | Vật che sát cảm biến (dưới ~2 cm số liệu không đáng tin). Bỏ vật che ra, hướng cảm biến ra khoảng trống. |
| Ảnh bị lật so với thực tế | Chỉnh "Lật ngang / Lật dọc / Đổi X↔Y" ở mục "Hướng ảnh". |
| Cổng USB tự rớt rồi tự cắm lại | Dấu hiệu tiếp xúc hoặc nguồn không ổn định. Thay cáp ngắn và tốt hơn, cắm vào cổng sau thùng máy (không qua hub), giữ dây Dupont chắc tay. Xem `journalctl -k \| grep usb`: lỗi `error -71` hoặc `Cannot enable` chỉ ra sự cố vật lý. |

**Vấn đề chưa kết luận:** trong quá trình phát triển đã gặp những lần cổng USB tự rớt và có lúc cảm biến không
phản hồi (`status=3`) gần nhau về thời gian. Phần mềm (server, firmware tự khôi phục) đã được làm bền để không
đứng, nhưng **nguyên nhân gốc chưa được xác định**; nghi ngờ nguồn cấp 3V3 hoặc tiếp xúc dây/cáp.

---

## Tham khảo
- Thư viện: https://github.com/RJRP44/VL53L8CX-Library (BSD-3-Clause)
- Tài liệu ST: UM3109 (hướng dẫn dùng VL53L8CX ULD) và datasheet VL53L8CX trên st.com
