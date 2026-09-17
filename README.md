# TorqMIT — Tool Verifikasi Torsi/Arus Motor AK40-10

Aplikasi desktop GUI untuk verifikasi torsi dan arus motor aktuator brushless **CubeMars AK40-10** (planetary gearbox 10:1, 14 pole pairs). Mendukung mode **UART Servo**, **CAN Servo**, dan **CAN MIT** (kontrol impedansi) dengan plotting real-time, step sweep otomatis, dan perekaman data CSV.

---

## Daftar Isi

- [Fitur](#fitur)
- [Spesifikasi Motor](#spesifikasi-motor)
- [Mode Komunikasi](#mode-komunikasi)
- [Arsitektur](#arsitektur)
- [Prasyarat](#prasyarat)
- [Instalasi](#instalasi)
- [Cara Pakai](#cara-pakai)
  - [Aplikasi Utama (GUI)](#aplikasi-utama-gui)
  - [Script Diagnostik CAN](#script-diagnostik-can)
  - [Script Tes MIT Kontinu](#script-tes-mit-kontinu)
- [Alur Kerja GUI](#alur-kerja-gui)
  - [Koneksi](#1-koneksi)
  - [Pemilihan Mode Input](#2-pemilihan-mode-input)
  - [Konfigurasi Parameter](#3-konfigurasi-parameter)
  - [Mengirim Perintah](#4-mengirim-perintah)
  - [Step Sweep Otomatis](#5-step-sweep-otomatis)
  - [Perekaman Data](#6-perekaman-data)
  - [Capture & Anotasi](#7-capture--anotasi)
  - [Emergency Stop](#8-emergency-stop)
- [Protokol MIT Mode](#protokol-mit-mode)
- [Rumus Utama](#rumus-utama)
- [Format Data CSV](#format-data-csv)
- [Struktur Proyek](#struktur-proyek)
- [Troubleshooting](#troubleshooting)
- [Referensi](#referensi)

---

## Fitur

- **3 interface komunikasi**: UART Serial (921600 baud), CAN Servo (extended frame), CAN MIT (standard frame)
- **Dual input mode**: Arus (A) atau Torsi (Nm) dengan konversi otomatis via Kt efektif
- **Dual-plot real-time**: Arus (A) dan Torsi (Nm) — target vs aktual, rolling window 30 detik di 20 Hz
- **Step sweep otomatis**: Start/end/step/dwell yang bisa dikonfigurasi, auto-capture di 80% waktu dwell
- **Perekaman data CSV**: 14 kolom dengan header metadata, otomatis mulai saat connect
- **Capture & anotasi**: Tandai titik data di grafik dengan label (target, aktual, error, torsi, kecepatan)
- **Auto-detect CAN**: Deteksi otomatis motor CAN ID dari broadcast servo
- **Auto-enter MIT mode**: Sekuens masuk MIT 5 percobaan dengan verifikasi
- **Ramping arus**: Ramp rate bisa dikonfigurasi (A/s atau Nm/s) untuk mencegah gerakan menyentak
- **E-STOP**: Shortcut tombol ESC, langsung nol-kan semua output
- **Mode simulasi**: Simulasi fisika lengkap dengan noise untuk testing tanpa hardware
- **Dark theme UI**: PySide6/Qt6 dengan stylesheet gelap
- **Serial monitor**: Monitor UART opsional bersamaan dengan komunikasi CAN
- **Telemetri langsung**: Arus, torsi, kecepatan, posisi, suhu, tegangan, kode error

---

## Spesifikasi Motor

| Parameter | Nilai | Keterangan |
|---|---|---|
| **Kt** | 0.056 Nm/A | Konstanta torsi (sisi rotor, sebelum gearbox) |
| **Gear Ratio** | 10:1 | Planetary gearbox built-in |
| **Efisiensi Gearbox** | 0.86 | Dihitung balik dari titik rated |
| **Kt Efektif** | 0.482 Nm/A | `0.056 x 10 x 0.86` — torsi per ampere di output shaft |
| **Pole Pairs** | 14 | 28 magnet (14 pasang N-S) |
| **Arus Rated** | 2.7 A | Arus kontinu maksimum |
| **Arus Peak** | 7.3 A | Arus sesaat maksimum |
| **Torsi Rated** | 1.3 Nm | Torsi kontinu di output shaft |
| **Torsi Peak** | 4.1 Nm | Torsi sesaat di output shaft |
| **Kecepatan Rated** | 435 rpm | Output shaft (= 60.900 ERPM) |
| **Encoder** | Magnetik 14-bit | Absolute, single-turn, inner ring |
| **Backlash** | 18 arcmin (0.3 deg) | Tidak terdeteksi encoder inner ring |

---

## Mode Komunikasi

| Mode | Interface | Kecepatan | Tipe Frame | Protokol |
|---|---|---|---|---|
| **UART Servo** | Serial USB | 40 Hz | Paket CRC16 | `COMM_SET_CURRENT (6)` — current loop |
| **CAN Servo** | socketCAN | 100 Hz | Extended CAN frame | `CAN_PACKET_SET_CURRENT (1)` — current loop |
| **CAN MIT** | socketCAN | 100 Hz | Standard CAN frame | Kontrol impedansi: `tau = kp*(p_des-p) + kd*(v_des-v) + t_ff` |

Aplikasi mendeteksi tipe interface secara otomatis dari pilihan dropdown:
- **COM port** (Windows) / **`/dev/ttyXXX`** (Linux) → UART Servo
- **`can0`**, **`vcan0`** → CAN bus (coba MIT dulu, fallback ke Servo)
- **Simulation** → Simulasi software dengan model fisika

---

## Arsitektur

```
+--------------------------------------------------------------------+
|                    Torsi_Encoder.py (PySide6 GUI)                   |
|  +------------+  +-------------------------------------+           |
|  |  App GUI   |  |     Plot Real-Time (pyqtgraph)      |           |
|  | - kontrol  |  |  Plot 1: Arus (A) Target vs Aktual  |           |
|  | - param    |  |  Plot 2: Torsi (Nm) Target vs Aktual |           |
|  | - sweep    |  +-------------------------------------+           |
|  | - rekam    |                                                    |
|  +-----+------+                                                    |
|        | QTimer 50ms                                               |
|  +-----v--------------------------------------------------+        |
|  |              Motor Controller                           |        |
|  |  +-----------------+      +-------------------------+   |        |
|  |  | Motor (UART)    |      |  MotorCAN (CAN bus)     |   |        |
|  |  | Thread 40 Hz    |      |  Thread 100 Hz          |   |        |
|  |  | Serial 921600   |      |  socketCAN 1 Mbps       |   |        |
|  |  | Frame CRC16     |      |  Mode MIT + SERVO       |   |        |
|  |  +--------+--------+      +------------+------------+   |        |
|  +-----------|-----------------------------|----------------+        |
+--------------|-----------------------------|------------------------+
               |                             |
               v                             v
      +----------------+          +--------------------+
      |  USB-UART      |          |  MCP2515 SPI-CAN   |
      |  (COM port)    |          |  (socketCAN)        |
      +-------+--------+          +---------+----------+
              |                             |
              v                             v
      +---------------------------------------------+
      |         Motor CubeMars AK40-10               |
      |   (14 PP, 10:1 gear, maks 4.1 Nm)           |
      +---------------------------------------------+

      Output data:  data/AK40-10_YYYY-MM-DD_HH-MM-SS.csv
```

### Ringkasan Class

| Class | Baris | Tanggung Jawab |
|---|---|---|
| `Motor` | 185–395 | Kontroler UART Servo mode. Thread background 40 Hz, frame CRC16, ramping arus, mode simulasi. |
| `MotorCAN` | 402–1083 | Kontroler CAN bus (Servo + MIT). Thread background 100 Hz, auto-detect motor ID, sekuens masuk/verifikasi MIT, filter echo MCP2515, serial monitor opsional, simulasi fisika. |
| `App` | 1089–1960 | GUI PySide6 (QMainWindow). Build UI, kelola koneksi, update grafik 20 Hz, automasi sweep, rekam CSV, capture/anotasi. |

---

## Prasyarat

- **Python** >= 3.10
- **OS**: Windows (UART) atau Linux (UART + CAN)
- **Hardware** (untuk testing motor asli):
  - Motor CubeMars AK40-10 + driver board
  - Adapter USB-UART (untuk mode UART) ATAU
  - MCP2515 SPI-CAN HAT di Raspberry Pi (untuk mode CAN)
  - Power supply DC 24V

> Fitur CAN bus (`can0`, MIT mode) membutuhkan **Linux dengan dukungan socketCAN**. Di Windows, hanya mode UART dan Simulasi yang tersedia.

---

## Instalasi

```bash
# Clone atau copy proyek
cd TorqMIT

# Install dependensi
pip install -r requirements.txt
```

### Dependensi

| Package | Versi | Fungsi |
|---|---|---|
| `PySide6` | >= 6.8.0 | Framework GUI Qt6 |
| `pyqtgraph` | >= 0.13.0 | Plotting real-time |
| `pyserial` | >= 3.5 | Komunikasi serial/UART |
| `numpy` | >= 2.0.0 | Komputasi numerik |
| `python-can` | >= 4.4.0 | Komunikasi CAN bus (opsional di Windows) |

### Setup CAN Bus (Khusus Linux/Raspberry Pi)

```bash
# Muat kernel module
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mcp251x   # untuk MCP2515 HAT

# Nyalakan interface CAN di 1 Mbps
sudo ip link set can0 up type can bitrate 1000000

# Verifikasi
ip -details link show can0

# (Opsional) Virtual CAN untuk testing tanpa hardware
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

---

## Cara Pakai

### Aplikasi Utama (GUI)

```bash
python Torsi_Encoder.py
```

Membuka GUI lengkap dengan panel koneksi, kontrol parameter, plot real-time, automasi sweep, dan perekaman CSV.

### Script Diagnostik CAN

```bash
# Khusus Linux, interface can0 harus aktif
python can_diag.py
```

Diagnostik CAN bus step-by-step (202 baris):

| Langkah | Aksi |
|---|---|
| 1 | Listen pasif (1 detik) — deteksi motor ID dari broadcast servo |
| 2 | Kirim perintah Servo current (0.5 A) via extended frame, lalu stop |
| 3 | MIT ENTER — hingga 3 percobaan (standard + extended frame) |
| 4 | MIT zero-torque — verifikasi mode MIT aktif |
| 5 | MIT 0.5 Nm — kirim torsi selama 2 detik, cetak feedback 10 Hz |
| 6 | MIT EXIT — shutdown bersih |

### Script Tes MIT Kontinu

```bash
# Khusus Linux, interface can0 harus aktif
python can_test_mit.py
```

Tes MIT mode 10 detik (148 baris):

| Fase | Durasi | Aksi |
|---|---|---|
| 1 | 0–5 detik | Zero torque — putar motor dengan tangan, amati feedback posisi/kecepatan |
| 2 | 5–10 detik | 0.5 Nm torsi dengan kd=0.5 — motor berputar ~1 rad/s |

Berjalan di 50 Hz, cetak setiap frame feedback ke-5, lapor jumlah TX/RX/no-reply.

---

## Alur Kerja GUI

### 1. Koneksi

```
[Dropdown Interface] -> [CAN ID] -> [Connect]
```

1. Klik **R** (refresh) untuk scan interface yang tersedia
2. Pilih interface dari dropdown:
   - `COMx` / `/dev/ttyUSBx` — mode UART Servo
   - `can0` / `vcan0` — CAN bus (otomatis masuk MIT, fallback ke Servo)
   - `Simulation` — simulasi software
3. Set **CAN ID** (default 2, set 0 untuk auto-detect)
4. Klik **Connect**

Untuk koneksi CAN, aplikasi menjalankan sekuens 4 langkah:
1. Auto-detect motor ID dari traffic broadcast servo
2. Kirim MIT EXIT untuk bersihkan state
3. MIT ENTER agresif (5 percobaan, filter echo MCP2515)
4. Verifikasi MIT dengan perintah zero-torque

Jika MIT gagal, otomatis fallback ke mode CAN Servo.

### 2. Pemilihan Mode Input

| Mode | Keterangan |
|---|---|
| **Arus (A)** | Kirim ampere langsung ke motor. Default 0.5 A. |
| **Torsi (Nm)** | Dikonversi ke ampere via `I = tau / 0.482`, lalu dikirim. |

Toggle lewat radio button di panel "Input Mode".

### 3. Konfigurasi Parameter

**Parameter UART Servo:**

| Param | Default | Satuan | Catatan |
|---|---|---|---|
| des P | 0.00 | deg | Disimpan, tidak dikirim di current loop |
| des S | 5000 | ERPM | Disimpan, tidak dikirim di current loop |
| des A | 30000 | ERPM/s^2 | Disimpan, tidak dikirim di current loop |
| Ramp | 1.0 | A/s | Aktif — mengontrol kecepatan ramp arus |

**Parameter MIT Control:**

| Param | Default | Satuan | Range | Catatan |
|---|---|---|---|---|
| Kp | 0.0 | - | 0–500 | Gain posisi |
| Kd | 0.5 | - | 0–5 | Gain damping kecepatan |
| p_des | 0.0 | rad | -12.5 s/d 12.5 | Posisi yang diinginkan |
| v_des | 0.0 | rad/s | -45.5 s/d 45.5 | Kecepatan yang diinginkan |
| Ramp | 0.5 | Nm/s | - | Kecepatan ramp torsi |

**Persamaan Torsi MIT:**
```
tau = kp * (p_des - p) + kd * (v_des - v) + t_ff
```

Untuk torsi murni dengan damping kecepatan, set `kp=0, v_des=0, kd>0`. Kecepatan maksimum = `t_ff / kd`.

**Setting MIT yang Direkomendasikan:**
| Kd | t_ff | Kecepatan Maks |
|---|---|---|
| 0.5 | 0.5 Nm | ~1 rad/s |
| 0.3 | 1.0 Nm | ~3.3 rad/s |

### 4. Mengirim Perintah

1. Masukkan nilai setpoint di kotak **Setpoint**
2. Klik **Send** — motor ramp ke target sesuai ramp rate yang dikonfigurasi
3. Klik **Stop** — motor ramp turun ke nol

Ramp mencegah lonjakan arus mendadak:
```
step = ramp_rate * dt
if abs(target - ramped) < step:
    ramped = target
else:
    ramped += step * sign(target - ramped)
```

### 5. Step Sweep Otomatis

```
[Start] [End] [Step] [Dwell(s)] -> [Run Sweep]
```

1. Konfigurasi parameter sweep:
   - **Start**: nilai setpoint pertama
   - **End**: nilai setpoint terakhir
   - **Step**: increment antar step
   - **Dwell**: waktu tahan per step (detik)
2. Klik **Run Sweep**
3. Aplikasi secara otomatis:
   - Generate urutan step
   - Kirim setiap step ke motor
   - Tahan selama waktu dwell
   - **Auto-capture** data di 80% waktu dwell (saat motor sudah settle)
   - Otomatis mulai rekam CSV jika belum aktif
   - Lanjut ke step berikutnya
4. Klik **Abort** untuk stop di tengah sweep

Contoh: sweep 0.1 sampai 0.5 A dengan step=0.1 dan dwell=5s:
```
Step 1: 0.100 A (tahan 5s, capture di 4.0s)
Step 2: 0.200 A (tahan 5s, capture di 4.0s)
Step 3: 0.300 A (tahan 5s, capture di 4.0s)
Step 4: 0.400 A (tahan 5s, capture di 4.0s)
Step 5: 0.500 A (tahan 5s, capture di 4.0s)
-> Sweep SELESAI!
```

### 6. Perekaman Data

- **Otomatis mulai**: rekaman dimulai otomatis saat connect
- **Toggle manual**: tombol Start/Stop recording
- File disimpan di folder `data/` dengan nama `AK40-10_YYYY-MM-DD_HH-MM-SS.csv`
- Penghitung baris ditampilkan selama rekaman
- Lihat [Format Data CSV](#format-data-csv) untuk detail

### 7. Capture & Anotasi

- Klik **Capture Now** untuk tandai titik data saat ini di grafik
- Tunggu ~2 detik agar motor settle, lalu snapshot
- Anotasi menampilkan: nilai target, nilai aktual, error %, torsi, kecepatan
- Titik hijau + label teks muncul di kedua plot

### 8. Emergency Stop

- Tekan tombol **ESC** atau klik tombol **E-STOP [ESC]**
- Langsung nol-kan semua output (arus/torsi di-set ke 0)
- Motor berhenti secepat mungkin
- Berfungsi di semua mode (UART, CAN Servo, CAN MIT)

---

## Protokol MIT Mode

MIT mode menggunakan standard CAN frame (bukan extended) dengan command 8-byte ter-pack:

### Frame Perintah (TX)

```
CAN ID: motor_id (standard frame)
Data [8 byte]:
  [0-1]  p_des    (16-bit, range -12.5 s/d 12.5 rad)
  [2-3]  v_des    (12-bit) | kp (4-bit atas)
  [4]    kp       (8-bit bawah)
  [5]    kd       (8-bit atas)
  [6]    kd (4-bit bawah) | t_ff (4-bit atas)
  [7]    t_ff     (8-bit bawah)
```

### Frame Balasan (RX)

```
Data [8 byte]:
  [0]    motor_id
  [1-2]  posisi    (16-bit, -12.5 s/d 12.5 rad)
  [3-4]  kecepatan (12-bit, -45.5 s/d 45.5 rad/s) | torsi (4-bit atas)
  [5]    torsi     (8-bit bawah, -5.0 s/d 5.0 Nm)
  [6]    suhu      (raw - 40 = Celsius)
  [7]    kode error
```

### Perintah Khusus

| Perintah | Data (8 byte) | Fungsi |
|---|---|---|
| **Masuk MIT** | `FF FF FF FF FF FF FF FC` | Pindahkan motor ke mode MIT |
| **Keluar MIT** | `FF FF FF FF FF FF FF FD` | Kembali ke mode Servo |
| **Set Nol** | `FF FF FF FF FF FF FF FE` | Set posisi saat ini sebagai titik nol |

---

## Rumus Utama

### Torsi di Output Shaft

```
tau_output = Iq * Kt * Gear_Ratio * Efisiensi_Gearbox
tau_output = Iq * 0.056 * 10 * 0.86
tau_output = Iq * 0.482
```

### Arus dari Torsi yang Diinginkan

```
Iq = tau_output / Kt_Efektif
Iq = tau_output / 0.482

Contoh: 0.5 Nm -> 0.5 / 0.482 = 1.038 A
```

### Konversi ERPM

```
ERPM = RPM_output * Pole_Pairs * Gear_Ratio
ERPM = RPM_output * 14 * 10
ERPM = RPM_output * 140

Contoh: 435 rpm (rated) = 435 * 140 = 60.900 ERPM
```

---

## Format Data CSV

File disimpan di `data/` dengan header metadata dan 14 kolom data.

### Header (diawali `#`)

```
# AK40-10 Torque/Current Verification Log
# Date: 2026-09-17 09:35:38
# Mode: MIT
# CAN ID: 2
# Kt_eff: 0.4816 Nm/A
# Gear Ratio: 10.0
# Input Mode: AMPS
# MIT Kp: 0.0
# MIT Kd: 0.5
# MIT v_des: 0.0
# MIT p_des: 0.0
```

### Kolom Data

| Kolom | Satuan | Keterangan |
|---|---|---|
| `time_s` | s | Waktu sejak connect |
| `target_current_A` | A | Arus yang diperintahkan |
| `actual_current_A` | A | Arus Iq terukur (feedback) |
| `target_torque_Nm` | Nm | Torsi target (dihitung) |
| `actual_torque_Nm` | Nm | Torsi aktual (dihitung) |
| `position_rad` | rad | Posisi motor (MIT) atau derajat (Servo) |
| `position_deg` | deg | Posisi dalam derajat |
| `speed_rad_s` | rad/s | Kecepatan sudut |
| `speed_rpm` | rpm | Kecepatan putar |
| `motor_temp_C` | C | Suhu kumparan motor |
| `mos_temp_C` | C | Suhu MOSFET/driver |
| `voltage_V` | V | Tegangan input |
| `error_code` | - | 0=OK, 1-7=fault (lihat manual) |
| `sweep_step` | - | Setpoint sweep saat ini (atau 0) |

---

## Struktur Proyek

```
TorqMIT/
├── Torsi_Encoder.py       # Aplikasi GUI utama (1998 baris)
│   ├── class Motor          # Kontroler UART Servo mode (40 Hz)
│   ├── class MotorCAN       # Kontroler CAN bus: Servo + MIT (100 Hz)
│   └── class App            # GUI PySide6, plot, sweep, rekam data
│
├── can_diag.py             # Tool diagnostik CAN bus via CLI (202 baris)
├── can_test_mit.py         # Tes MIT mode kontinu via CLI (148 baris)
├── requirements.txt        # Dependensi Python
├── README.md               # File ini
│
└── data/                   # Log data CSV (otomatis di-generate)
    └── AK40-10_YYYY-MM-DD_HH-MM-SS.csv
```

---

## Troubleshooting

### UART

| Masalah | Solusi |
|---|---|
| Port tidak muncul di daftar | Klik **R** untuk refresh. Cek koneksi kabel USB. |
| Tidak ada data telemetri | Pastikan baudrate 921600. Cek wiring TX/RX (TX->RX, RX->TX). |
| Motor tidak merespons | Pastikan motor dalam **Servo mode** via CubeMarsTool. Cek power supply (24V). |

### CAN Bus

| Masalah | Solusi |
|---|---|
| `can0` tidak muncul di daftar | Jalankan `sudo ip link set can0 up type can bitrate 1000000` |
| Motor ID tidak terdeteksi | Cek wiring CAN_H/CAN_L. Pastikan motor menyala. Coba set ID manual (default: 2). |
| MIT mode gagal | Motor harus dipindahkan ke firmware MIT dulu via CubeMarsTool. Aplikasi akan fallback ke mode Servo. |
| Frame echo (MCP2515) | Perilaku normal — aplikasi otomatis mem-filter frame yang dikirim sendiri. |
| `python-can` tidak ditemukan | Install dengan `pip install python-can`. Hanya diperlukan untuk mode CAN. |

### Umum

| Masalah | Solusi |
|---|---|
| Motor berputar kencang di current loop | Normal — current loop tidak punya speed limiter. Gunakan MIT mode (`kd > 0`) untuk damping kecepatan. |
| Nilai torsi terlihat tidak sesuai | Torsi dihitung (`Iq * 0.482`), bukan diukur langsung. Verifikasi dengan torque sensor untuk akurasi. |
| Motor tidak bergerak di bawah 0.5 A | Gesekan breakaway gearbox ~0.241 Nm (~0.5 A). Naikkan arus. |

---

## Referensi

- **AK Series Driver Manual V1.0.18** — CubeMars (spesifikasi protokol, bagian 5.1–5.3)
- **Datasheet AK40-10** — CubeMars (konstanta motor, parameter elektrik)
- **Instruksi Instalasi Driver AK40-2410-1A-A1** — CubeMars
- [Website Resmi CubeMars](https://www.cubemars.com/)
