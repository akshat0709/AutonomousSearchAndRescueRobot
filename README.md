# 🤖 Rescue Bot — Raspberry Pi Autonomous Robot

A web-controlled, camera-equipped rescue robot built on a Raspberry Pi. Features live video streaming, full manual drive control, autonomous obstacle-avoidance mode with left/right scanning, a 4-servo robotic arm, and real-time telemetry (temperature, humidity, flame detection, human detection).

---

## ✨ Features

| Feature | Details |
|---|---|
| 🎥 **Live Video Stream** | MJPEG stream from Pi Camera via `picamera2` |
| 🕹️ **Manual Drive Control** | Web D-pad + WASD keyboard shortcuts |
| 🤖 **Autonomous Mode** | Scan left → scan right → decide, then navigate |
| 🦾 **Robotic Arm** | 4 servo joints (Camera Pan, Arm Base, Arm Link, Gripper) |
| 🌡️ **DHT11 Sensor** | Real-time temperature & humidity telemetry |
| 🔥 **Flame Sensor** | Digital flame/fire alert |
| 👤 **Human Detection** | HOG-based person detection via OpenCV |
| 📡 **Ultrasonic Sensor** | HC-SR04 obstacle distance measurement (gpiozero or GPIO fallback) |
| 💻 **Web Dashboard** | Glassmorphism dark-mode UI, no app install required |

---

## 📸 Dashboard Preview

> Access the dashboard from any device on your local network at `http://<raspberry-pi-ip>:5050`

The dashboard is split into three panels:
- **Left** — Live camera feed + Auto/Stop Auto buttons
- **Center** — D-pad manual controls + real-time telemetry cards
- **Right** — Individual servo controls for the robotic arm

---

## 🔧 Hardware Requirements

- Raspberry Pi (3B+ / 4 / 5 recommended)
- Pi Camera Module (v1/v2/HQ)
- L298N or similar dual H-bridge motor driver
- 2× DC motors (for differential drive)
- HC-SR04 ultrasonic distance sensor
- DHT11 temperature & humidity sensor
- Flame / fire sensor module (digital output)
- 4× SG90 (or equivalent) servo motors
- Power supply suitable for Pi + motors

---

## 📌 GPIO Pin Map

| Component | GPIO (BCM) |
|---|---|
| Motor IN1 | 17 |
| Motor IN2 | 27 |
| Motor IN3 | 22 |
| Motor IN4 | 23 |
| Ultrasonic TRIG | 5 |
| Ultrasonic ECHO | 6 |
| DHT11 Data | 25 |
| Flame Sensor | 24 |
| Servo 1 — Camera Pan | 12 |
| Servo 2 — Arm Base | 13 |
| Servo 3 — Arm Link | 19 |
| Servo 4 — Gripper | 26 |

---

## 🚀 Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/<your-username>/rescue-bot.git
cd rescue-bot
```

### 2. Install dependencies

```bash
sudo apt update
sudo apt install python3-pip python3-opencv -y

pip install flask picamera2 RPi.GPIO gpiozero dht11
```

> **Note:** `gpiozero` provides significantly more accurate ultrasonic readings on Raspberry Pi. The code automatically falls back to a software-timed method if it is not available.

### 3. Run the server

```bash
python3 control_cam.py
```

### 4. Open the dashboard

Open a browser on any device on the same network and navigate to:

```
http://<raspberry-pi-ip>:5050
```

---

## 🧠 Autonomous Mode — How It Works

When an obstacle is detected **within 20 cm**, the robot performs a scan sequence before turning:

```
Obstacle detected
      │
      ▼
    STOP
      │
      ├─► Look LEFT  (servo → 30°)  → measure distance
      │
      ├─► Look RIGHT (servo → 150°) → measure distance
      │
      ├─► Re-centre camera (servo → 90°)
      │
      └─► Turn toward the CLEARER side
              • Left > Right  → turn LEFT
              • Right > Left  → turn RIGHT
              • Equal         → random 50/50
```

This prevents the robot from always blindly turning the same direction and getting stuck.

---

## 🌐 API Endpoints

| Endpoint | Description |
|---|---|
| `GET /` | Web dashboard |
| `GET /video` | MJPEG camera stream |
| `GET /forward` | Drive forward |
| `GET /back` | Drive backward |
| `GET /left` | Turn left |
| `GET /right` | Turn right |
| `GET /stop` | Stop motors |
| `GET /auto` | Start autonomous mode |
| `GET /stop_auto` | Stop autonomous mode |
| `GET /servo/<id>/<angle>` | Move servo (id: 1–4, angle: 0–180) |
| `GET /status` | JSON telemetry snapshot |

### `/status` response example

```json
{
  "distance": 34.5,
  "human": false,
  "mode": "AUTO",
  "temp": 27.3,
  "humidity": 58.0,
  "flame": false
}
```

---

## 🗂️ Project Structure

```
rescue-bot/
├── control_cam.py   # Main Flask app — sensors, motors, servos, camera, routes
└── README.md
```

---

## ⌨️ Keyboard Shortcuts

| Key | Action |
|---|---|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| *(release)* | Stop |

---

## 🛠️ Troubleshooting

| Issue | Fix |
|---|---|
| Camera won't start | Run `sudo raspi-config` → enable camera interface |
| DHT11 always `-` | Check wiring and pull-up resistor on data pin |
| Ultrasonic readings erratic | Ensure stable 5V power; `gpiozero` mode is more reliable |
| Servos jittering | PWM duty cycle is reset to 0 after each move to stop jitter — normal behaviour |
| Dashboard not loading | Make sure port 5050 is not blocked by a firewall; confirm Pi IP with `hostname -I` |

---

## 📄 License

MIT License — free to use, modify, and share.

---

## 🙌 Contributing

Pull requests are welcome! For major changes, please open an issue first to discuss what you'd like to change.
