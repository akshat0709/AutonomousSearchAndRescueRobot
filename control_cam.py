from flask import Flask, Response, render_template_string, jsonify
from picamera2 import Picamera2
import cv2
import RPi.GPIO as GPIO
import time
import threading
import numpy as np

try:
    from gpiozero import DistanceSensor
    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False
    print("gpiozero not found. Install with: sudo pip install gpiozero")

try:
    import dht11
    DHT_AVAILABLE = True
except ImportError:
    dht11 = None
    DHT_AVAILABLE = False
    print("dht11 library not found. DHT11 sensor will not work. (pip install dht11)")

app = Flask(__name__)

# ---------------- GPIO PINS ----------------
IN1, IN2, IN3, IN4 = 17, 27, 22, 23
TRIG, ECHO = 5, 6
DHT_PIN = 25
FLAME_PIN = 24

# Servo Pins
SERVO1_PIN = 12 # Camera Movement
SERVO2_PIN = 13 # Arm Base
SERVO3_PIN = 19 # Arm Link
SERVO4_PIN = 26 # Arm Gripper

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor setup
for p in [IN1, IN2, IN3, IN4]:
    GPIO.setup(p, GPIO.OUT)

# Ultrasonic setup — only needed if gpiozero is not available
if not GPIOZERO_AVAILABLE:
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

# Flame sensor setup
GPIO.setup(FLAME_PIN, GPIO.IN)

# DHT11 instance (created once, reused on every read)
if DHT_AVAILABLE:
    dht_instance = dht11.DHT11(pin=DHT_PIN)
else:
    dht_instance = None

# gpiozero DistanceSensor (TRIG=5, ECHO=6, BCM numbering)
# max_distance=4 means 4 meters
if GPIOZERO_AVAILABLE:
    try:
        ultrasonic = DistanceSensor(echo=ECHO, trigger=TRIG, max_distance=4)
    except Exception as e:
        ultrasonic = None
        print(f"Failed to init DistanceSensor: {e}")
else:
    ultrasonic = None

# Servo setup
servo_pins = [SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN]
for p in servo_pins:
    GPIO.setup(p, GPIO.OUT)

pwm_servos = {}
for p in servo_pins:
    pwm = GPIO.PWM(p, 50) # 50Hz for servo
    pwm.start(0)
    pwm_servos[p] = pwm

# ---------------- GLOBAL STATE ----------------
auto_running = False
current_distance = 0
current_temp = None
current_humidity = None
flame_detected = False
human_detected = False

# ---------------- MOTOR ----------------
def forward():
    GPIO.output(IN1,1); GPIO.output(IN2,0)
    GPIO.output(IN3,1); GPIO.output(IN4,0)

def back():
    GPIO.output(IN1,0); GPIO.output(IN2,1)
    GPIO.output(IN3,0); GPIO.output(IN4,1)

def left():
    GPIO.output(IN1,0); GPIO.output(IN2,1)
    GPIO.output(IN3,1); GPIO.output(IN4,0)

def right():
    GPIO.output(IN1,1); GPIO.output(IN2,0)
    GPIO.output(IN3,0); GPIO.output(IN4,1)

def stop():
    GPIO.output(IN1,0); GPIO.output(IN2,0)
    GPIO.output(IN3,0); GPIO.output(IN4,0)

# ---------------- SERVO CONTROL ----------------
def set_servo_angle(pin, angle):
    if pin in pwm_servos:
        # 0 to 180 degrees mapping to duty cycle ~2.5 to 12.5
        duty = 2.5 + (angle / 180.0) * 10.0
        pwm_servos[pin].ChangeDutyCycle(duty)
        time.sleep(0.3)
        pwm_servos[pin].ChangeDutyCycle(0) # Stop sending PWM to prevent jitter

# ---------------- ULTRASONIC ----------------
def get_distance():
    """Read distance in cm. Uses gpiozero's DistanceSensor if available
    (kernel-timed, far more accurate on Raspberry Pi), else falls back
    to a median-filtered software loop."""
    if ultrasonic is not None:
        try:
            dist_m = ultrasonic.distance  # returns meters, blocking until measured
            if dist_m is None:
                return None
            dist_cm = dist_m * 100.0
            if 2 <= dist_cm <= 400:
                return round(dist_cm, 1)
            return None
        except Exception:
            return None

    # ---- Fallback: manual timing with median filter ----
    def _raw():
        GPIO.output(TRIG, False)
        time.sleep(0.002)
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        t0 = time.time()
        timeout = t0 + 0.04

        pulse_start = t0
        while GPIO.input(ECHO) == 0 and time.time() < timeout:
            pulse_start = time.time()

        pulse_end = pulse_start
        while GPIO.input(ECHO) == 1 and time.time() < timeout:
            pulse_end = time.time()

        if pulse_end > pulse_start:
            return (pulse_end - pulse_start) * 17150
        return None

    readings = []
    for _ in range(5):
        d = _raw()
        if d is not None and 2 <= d <= 400:
            readings.append(d)
        time.sleep(0.015)

    if readings:
        readings.sort()
        return round(readings[len(readings) // 2], 1)
    return None

# ---------------- DHT11 LOOP (isolated thread for timing accuracy) ----------------
def dht_loop():
    """Runs DHT11 reads in its own thread so that ultrasonic/GPIO activity
    in sensor_loop cannot disturb the microsecond-level DHT11 timing."""
    global current_temp, current_humidity
    while True:
        if DHT_AVAILABLE and dht_instance is not None:
            try:
                result = dht_instance.read()
                if result.is_valid():
                    current_temp = round(result.temperature, 1)
                    current_humidity = round(result.humidity, 1)
            except Exception as e:
                print(f"DHT11 read error: {e}")
        time.sleep(2)  # DHT11 needs at least 1s between reads

# ---------------- SENSOR & AUTO LOOP ----------------
def sensor_loop():
    global auto_running, current_distance, flame_detected

    while True:
        # Always read distance and flame
        d = get_distance()
        if d is not None:
            current_distance = round(d, 2)

        # Read Flame (Low means flame detected for typical modules)
        flame_detected = (GPIO.input(FLAME_PIN) == GPIO.LOW)

        # Handle AUTO mode logic
        if auto_running:
            if current_distance and current_distance < 20:
                stop()
                time.sleep(0.3)

                # --- Scan LEFT ---
                set_servo_angle(SERVO1_PIN, 30)   # look left
                time.sleep(0.4)                    # wait for servo & sensor to settle
                dist_left = get_distance() or 0

                # --- Scan RIGHT ---
                set_servo_angle(SERVO1_PIN, 150)  # look right
                time.sleep(0.4)
                dist_right = get_distance() or 0

                # --- Return camera to centre ---
                set_servo_angle(SERVO1_PIN, 90)
                time.sleep(0.2)

                # --- Decide which way to turn ---
                import random
                if dist_left > dist_right:
                    left()
                    time.sleep(0.7)
                elif dist_right > dist_left:
                    right()
                    time.sleep(0.7)
                else:
                    # Both sides equally blocked — pick randomly
                    if random.random() < 0.5:
                        left()
                    else:
                        right()
                    time.sleep(0.7)

                stop()
                time.sleep(0.1)
            else:
                forward()
        else:
            # We don't auto-stop in manual mode unless we want obstacle avoidance
            pass

        time.sleep(0.1)

# ---------------- HUMAN DETECTION ----------------
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# ---------------- CAMERA ----------------
picam2 = Picamera2()
try:
    picam2.configure(picam2.create_preview_configuration(main={"size": (640,480)}))
    picam2.start()
except Exception as e:
    print(f"Failed to start camera: {e}")

def gen_frames():
    global human_detected

    while True:
        try:
            frame = picam2.capture_array()
        except Exception:
            # Create dummy frame if camera fails
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            time.sleep(1)

        # FIX format
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        frame = cv2.resize(frame, (400,300))

        rects, _ = hog.detectMultiScale(frame, winStride=(4,4),
                                        padding=(8,8), scale=1.05)

        human_detected = len(rects) > 0

        for (x,y,w,h) in rects:
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(frame,"Human",(x,y-5),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# ---------------- ROUTES ----------------
@app.route('/')
def home():
    return render_template_string(html)

@app.route('/video')
def video():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/forward')
def f():
    global auto_running
    auto_running = False
    forward()
    return "OK"

@app.route('/back')
def b():
    global auto_running
    auto_running = False
    back()
    return "OK"

@app.route('/left')
def l():
    global auto_running
    auto_running = False
    left()
    return "OK"

@app.route('/right')
def r():
    global auto_running
    auto_running = False
    right()
    return "OK"

@app.route('/stop')
def s():
    stop()
    return "OK"

@app.route('/auto')
def auto():
    global auto_running
    auto_running = True
    return "OK"

@app.route('/stop_auto')
def stop_auto():
    global auto_running
    auto_running = False
    stop()
    return "OK"

@app.route('/servo/<int:sid>/<int:angle>')
def servo_control(sid, angle):
    # Mapping servo ID from front-end to GPIO pin
    pin_map = {
        1: SERVO1_PIN,
        2: SERVO2_PIN,
        3: SERVO3_PIN,
        4: SERVO4_PIN
    }
    if sid in pin_map:
        # Limit angle for safety
        angle = max(0, min(180, angle))
        set_servo_angle(pin_map[sid], angle)
    return "OK"

@app.route('/status')
def status():
    return jsonify({
        "distance": current_distance,
        "human": human_detected,
        "mode": "AUTO" if auto_running else "MANUAL",
        "temp": current_temp,
        "humidity": current_humidity,
        "flame": flame_detected
    })

# ---------------- HTML ----------------
html = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Rescue Bot Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&display=swap" rel="stylesheet">
<link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css" rel="stylesheet">
<style>
    :root {
        --bg-gradient: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
        --glass-bg: rgba(255, 255, 255, 0.05);
        --glass-border: rgba(255, 255, 255, 0.1);
        --primary: #3b82f6;
        --primary-hover: #2563eb;
        --danger: #ef4444;
        --danger-hover: #dc2828;
        --success: #10b981;
        --text: #f8fafc;
        --text-muted: #94a3b8;
    }

    * { box-sizing: border-box; margin: 0; padding: 0; }

    body {
        font-family: 'Inter', sans-serif;
        background: var(--bg-gradient);
        color: var(--text);
        min-height: 100vh;
        display: flex;
        flex-direction: column;
        align-items: center;
        padding: 1rem;
    }

    .header { text-align: center; margin-bottom: 1rem; }
    .header h1 {
        font-weight: 800; font-size: 2rem;
        background: linear-gradient(to right, #60a5fa, #3b82f6);
        -webkit-background-clip: text; -webkit-text-fill-color: transparent;
        display: flex; align-items: center; gap: 0.5rem; justify-content: center;
    }
    .header p { color: var(--text-muted); margin-top: 0.2rem; font-size: 1rem; }

    .dashboard {
        display: grid;
        grid-template-columns: 1fr 320px 320px;
        gap: 1.5rem;
        max-width: 1300px;
        width: 100%;
    }

    @media (max-width: 1000px) {
        .dashboard { grid-template-columns: 1fr 1fr; }
    }
    @media (max-width: 700px) {
        .dashboard { grid-template-columns: 1fr; }
    }

    .panel {
        background: var(--glass-bg); border: 1px solid var(--glass-border);
        border-radius: 16px; padding: 1.25rem; backdrop-filter: blur(10px);
        box-shadow: 0 4px 20px rgba(0, 0, 0, 0.2);
    }

    .video-container {
        position: relative; border-radius: 12px; overflow: hidden;
        aspect-ratio: 4/3; background: #000;
        display: flex; align-items: center; justify-content: center;
        box-shadow: 0 4px 15px rgba(0,0,0,0.5); border: 1px solid var(--glass-border);
    }
    .video-container img { width: 100%; height: 100%; object-fit: cover; }

    .status-overlay {
        position: absolute; top: 1rem; left: 1rem; background: rgba(0, 0, 0, 0.6);
        padding: 0.5rem 1rem; border-radius: 8px; backdrop-filter: blur(4px);
        font-size: 0.8rem; font-weight: 600; display: flex; align-items: center; gap: 0.5rem;
        border: 1px solid var(--glass-border);
    }

    .status-dot {
        width: 10px; height: 10px; background: var(--success);
        border-radius: 50%; box-shadow: 0 0 10px var(--success);
        animation: pulse 2s infinite;
    }

    @keyframes pulse {
        0% { transform: scale(0.95); box-shadow: 0 0 0 0 rgba(16, 185, 129, 0.7); }
        70% { transform: scale(1); box-shadow: 0 0 0 10px rgba(16, 185, 129, 0); }
        100% { transform: scale(0.95); box-shadow: 0 0 0 0 rgba(16, 185, 129, 0); }
    }

    .dpad {
        display: grid; grid-template-columns: repeat(3, 1fr); grid-template-rows: repeat(3, 1fr);
        gap: 0.5rem; width: 180px; margin: 0 auto;
    }

    .btn {
        background: rgba(255, 255, 255, 0.1); border: 1px solid var(--glass-border);
        color: var(--text); border-radius: 12px; cursor: pointer;
        display: flex; align-items: center; justify-content: center;
        font-size: 1.25rem; transition: all 0.2s ease; aspect-ratio: 1;
        user-select: none;
    }
    .btn:hover { background: rgba(255, 255, 255, 0.2); transform: translateY(-2px); }
    .btn:active { transform: translateY(2px); }
    .btn.up { grid-column: 2; grid-row: 1; }
    .btn.left { grid-column: 1; grid-row: 2; }
    .btn.stop { grid-column: 2; grid-row: 2; background: rgba(239, 68, 68, 0.2); color: var(--danger); border-color: rgba(239, 68, 68, 0.5); }
    .btn.right { grid-column: 3; grid-row: 2; }
    .btn.down { grid-column: 2; grid-row: 3; }

    .action-buttons { display: flex; gap: 1rem; justify-content: center; margin-top: 1rem; }
    .btn-action {
        padding: 0.75rem 1rem; border-radius: 8px; font-weight: 600; font-size: 0.9rem;
        border: none; cursor: pointer; transition: all 0.2s;
        display: flex; align-items: center; gap: 0.5rem; flex: 1; justify-content: center;
    }
    .btn-auto { background: var(--primary); color: white; }
    .btn-auto:hover { background: var(--primary-hover); }
    .btn-stop-auto { background: rgba(255,255,255,0.1); color: white; border: 1px solid var(--glass-border); }
    .btn-stop-auto:hover { background: rgba(239, 68, 68, 0.8); }

    .telemetry { display: flex; flex-direction: column; gap: 0.75rem; }
    .telemetry-card {
        background: rgba(0, 0, 0, 0.2); border-radius: 10px; padding: 0.75rem 1rem;
        display: flex; align-items: center; justify-content: space-between;
        border: 1px solid var(--glass-border);
    }
    .telemetry-label { color: var(--text-muted); font-size: 0.85rem; display: flex; align-items: center; gap: 0.5rem; }
    .telemetry-value { font-weight: 600; font-size: 1rem; }
    
    .value-highlight { color: var(--primary); }
    .value-warning { color: #f59e0b; }
    .value-danger { color: var(--danger); }
    .value-success { color: var(--success); }

    .servo-controls { display: flex; flex-direction: column; gap: 1rem; }
    .servo-group { display: flex; flex-direction: column; gap: 0.25rem; }
    .servo-header { display: flex; justify-content: space-between; align-items: center; font-size: 0.85rem; color: var(--text); }
    .servo-value { color: var(--primary); font-weight: 600; font-variant-numeric: tabular-nums; }
</style>
</head>
<body>

<div class="header">
    <h1><i class="fa-solid fa-robot"></i> Rescue Bot</h1>
    <p>Advanced Command & Control Center</p>
</div>

<div class="dashboard">
    <!-- Column 1: Video -->
    <div class="panel">
        <div class="video-container">
            <img src="/video" alt="Camera Feed">
            <div class="status-overlay">
                <div class="status-dot"></div>
                Live Feed
            </div>
        </div>
        <div class="action-buttons">
            <button class="btn-action btn-auto" onclick="startAuto()">
                <i class="fa-solid fa-wand-magic-sparkles"></i> Auto Mode
            </button>
            <button class="btn-action btn-stop-auto" onclick="stopAuto()">
                <i class="fa-solid fa-hand"></i> Stop Auto
            </button>
        </div>
    </div>

    <!-- Column 2: Drive & Telemetry -->
    <div style="display: flex; flex-direction: column; gap: 1.5rem;">
        <div class="panel">
            <h3 style="margin-bottom: 1rem; text-align: center; color: var(--text-muted); font-size: 1.1rem;">
                <i class="fa-solid fa-gamepad"></i> Manual Override
            </h3>
            <div class="dpad">
                <button class="btn up" onclick="send('forward')" title="Forward (W)"><i class="fa-solid fa-chevron-up"></i></button>
                <button class="btn left" onclick="send('left')" title="Left (A)"><i class="fa-solid fa-chevron-left"></i></button>
                <button class="btn stop" onclick="send('stop')" title="Stop"><i class="fa-solid fa-stop"></i></button>
                <button class="btn right" onclick="send('right')" title="Right (D)"><i class="fa-solid fa-chevron-right"></i></button>
                <button class="btn down" onclick="send('back')" title="Back (S)"><i class="fa-solid fa-chevron-down"></i></button>
            </div>
        </div>

        <div class="panel telemetry" style="flex: 1;">
            <h3 style="margin-bottom: 0.5rem; color: var(--text-muted); font-size: 1.1rem;">
                <i class="fa-solid fa-satellite-dish"></i> Telemetry
            </h3>
            <div class="telemetry-card">
                <div class="telemetry-label"><i class="fa-solid fa-microchip"></i> Mode</div>
                <div class="telemetry-value" id="mode">-</div>
            </div>
            <div class="telemetry-card">
                <div class="telemetry-label"><i class="fa-solid fa-ruler"></i> Distance</div>
                <div class="telemetry-value"><span id="distance">-</span> cm</div>
            </div>
            <div class="telemetry-card">
                <div class="telemetry-label"><i class="fa-solid fa-thermometer-half"></i> Env</div>
                <div class="telemetry-value"><span id="temp">-</span>°C / <span id="humid">-</span>%</div>
            </div>
            <div class="telemetry-card">
                <div class="telemetry-label"><i class="fa-solid fa-fire"></i> Flame</div>
                <div class="telemetry-value" id="flame">-</div>
            </div>
            <div class="telemetry-card">
                <div class="telemetry-label"><i class="fa-solid fa-person-rays"></i> Human</div>
                <div class="telemetry-value" id="human">-</div>
            </div>
        </div>
    </div>

    <!-- Column 3: Servos -->
    <div class="panel">
        <h3 style="margin-bottom: 1.5rem; color: var(--text-muted); font-size: 1.1rem; text-align: center;">
            <i class="fa-solid fa-robot"></i> Robotic Arm
        </h3>
        <div class="servo-controls">
            <!-- Servo 1 -->
            <div class="servo-group">
                <div class="servo-header">
                    <span><i class="fa-solid fa-camera"></i> Camera Pan</span>
                    <span class="servo-value" id="val-s1">90°</span>
                </div>
                <div style="display: flex; justify-content: center; gap: 15px; margin-top: 5px;">
                    <button class="btn left" style="width: 50px; height: 50px; margin: 0;" onmousedown="startServo(1, -5)" onmouseup="stopServo()" onmouseleave="stopServo()" ontouchstart="startServo(1, -5)" ontouchend="stopServo()"><i class="fa-solid fa-chevron-left"></i></button>
                    <button class="btn right" style="width: 50px; height: 50px; margin: 0;" onmousedown="startServo(1, 5)" onmouseup="stopServo()" onmouseleave="stopServo()" ontouchstart="startServo(1, 5)" ontouchend="stopServo()"><i class="fa-solid fa-chevron-right"></i></button>
                </div>
            </div>
            <!-- Servo 2 -->
            <div class="servo-group">
                <div class="servo-header">
                    <span><i class="fa-solid fa-hand-fist"></i> Arm Base</span>
                    <span class="servo-value" id="val-s2">90°</span>
                </div>
                <div style="display: flex; justify-content: center; gap: 15px; margin-top: 5px;">
                    <button class="btn left" style="width: 50px; height: 50px; margin: 0;" onmousedown="startServo(2, -5)" onmouseup="stopServo()" onmouseleave="stopServo()" ontouchstart="startServo(2, -5)" ontouchend="stopServo()"><i class="fa-solid fa-chevron-left"></i></button>
                    <button class="btn right" style="width: 50px; height: 50px; margin: 0;" onmousedown="startServo(2, 5)" onmouseup="stopServo()" onmouseleave="stopServo()" ontouchstart="startServo(2, 5)" ontouchend="stopServo()"><i class="fa-solid fa-chevron-right"></i></button>
                </div>
            </div>
            <!-- Servo 3 -->
            <div class="servo-group">
                <div class="servo-header">
                    <span><i class="fa-solid fa-hand-sparkles"></i> Arm Link</span>
                    <span class="servo-value" id="val-s3">90°</span>
                </div>
                <div style="display: flex; justify-content: center; gap: 15px; margin-top: 5px;">
                    <button class="btn up" style="width: 50px; height: 50px; margin: 0;" onmousedown="startServo(3, -5)" onmouseup="stopServo()" onmouseleave="stopServo()" ontouchstart="startServo(3, -5)" ontouchend="stopServo()"><i class="fa-solid fa-chevron-up"></i></button>
                    <button class="btn down" style="width: 50px; height: 50px; margin: 0;" onmousedown="startServo(3, 5)" onmouseup="stopServo()" onmouseleave="stopServo()" ontouchstart="startServo(3, 5)" ontouchend="stopServo()"><i class="fa-solid fa-chevron-down"></i></button>
                </div>
            </div>
            <!-- Servo 4 -->
            <div class="servo-group">
                <div class="servo-header">
                    <span><i class="fa-solid fa-hand-holding"></i> Arm Gripper</span>
                    <span class="servo-value" id="val-s4">90°</span>
                </div>
                <div style="display: flex; justify-content: center; gap: 15px; margin-top: 5px;">
                    <button class="btn left" style="width: 50px; height: 50px; margin: 0;" onmousedown="startServo(4, -5)" onmouseup="stopServo()" onmouseleave="stopServo()" ontouchstart="startServo(4, -5)" ontouchend="stopServo()"><i class="fa-solid fa-compress"></i></button>
                    <button class="btn right" style="width: 50px; height: 50px; margin: 0;" onmousedown="startServo(4, 5)" onmouseup="stopServo()" onmouseleave="stopServo()" ontouchstart="startServo(4, 5)" ontouchend="stopServo()"><i class="fa-solid fa-expand"></i></button>
                </div>
            </div>
        </div>
    </div>
</div>

<script>
function send(cmd){ fetch('/' + cmd); }
function startAuto(){ fetch('/auto'); }
function stopAuto(){ fetch('/stop_auto'); }

// Servo continuous pressing
let servoAngles = {1: 90, 2: 90, 3: 90, 4: 90};
let servoInterval = null;
let servoTimeout = null;

function startServo(id, delta) {
    if(servoInterval) clearInterval(servoInterval);
    stepServoAmount(id, delta);
    servoInterval = setInterval(() => {
        stepServoAmount(id, delta);
    }, 100); // adjust speed of continuous movement here
}

function stopServo() {
    if(servoInterval) clearInterval(servoInterval);
}

function stepServoAmount(id, delta) {
    servoAngles[id] += delta;
    if(servoAngles[id] > 180) servoAngles[id] = 180;
    if(servoAngles[id] < 0) servoAngles[id] = 0;
    
    document.getElementById('val-s' + id).innerText = servoAngles[id] + '°';
    
    if(servoTimeout) return; // throttle requests
    servoTimeout = setTimeout(() => {
        fetch('/servo/' + id + '/' + servoAngles[id]);
        servoTimeout = null;
    }, 80);
}

setInterval(() => {
    fetch('/status')
    .then(r => r.json())
    .then(data => {
        const modeEl = document.getElementById('mode');
        modeEl.innerText = data.mode;
        modeEl.className = 'telemetry-value ' + (data.mode === 'AUTO' ? 'value-highlight' : '');

        const dist = data.distance;
        const distEl = document.getElementById('distance');
        distEl.innerText = dist !== null ? dist : '-';
        if (dist !== null && dist < 20) {
            distEl.parentElement.className = 'telemetry-value value-danger';
        } else if (dist !== null && dist < 50) {
            distEl.parentElement.className = 'telemetry-value value-warning';
        } else {
            distEl.parentElement.className = 'telemetry-value';
        }

        document.getElementById('temp').innerText = data.temp !== null ? data.temp : '-';
        document.getElementById('humid').innerText = data.humidity !== null ? data.humidity : '-';

        const flameEl = document.getElementById('flame');
        if(data.flame) {
            flameEl.innerHTML = '<span class="value-danger"><i class="fa-solid fa-fire-flame-curved"></i> ALERT</span>';
        } else {
            flameEl.innerHTML = '<span class="value-success">SAFE</span>';
        }

        const humanEl = document.getElementById('human');
        if (data.human) {
            humanEl.innerHTML = '<span class="value-danger"><i class="fa-solid fa-triangle-exclamation"></i> DETECTED</span>';
        } else {
            humanEl.innerHTML = '<span class="value-success">CLEAR</span>';
        }
    })
    .catch(err => console.error("Error fetching status:", err));
}, 500);

document.addEventListener('keydown', e=>{
    if(e.repeat) return;
    if(e.key==='w' || e.key==='W') send('forward');
    if(e.key==='s' || e.key==='S') send('back');
    if(e.key==='a' || e.key==='A') send('left');
    if(e.key==='d' || e.key==='D') send('right');
});

document.addEventListener('keyup', e=>{
    const key = e.key.toLowerCase();
    if(['w','a','s','d'].includes(key)) send('stop');
});

// Prevent right click menu on buttons when long pressing on touch devices
document.querySelectorAll('.btn').forEach(b => {
    b.addEventListener('contextmenu', e => e.preventDefault());
});
</script>

</body>
</html>
"""

# ---------------- MAIN ----------------
if __name__ == '__main__':
    t = threading.Thread(target=sensor_loop)
    t.daemon = True
    t.start()

    dht_thread = threading.Thread(target=dht_loop)
    dht_thread.daemon = True
    dht_thread.start()

    app.run(host='0.0.0.0', port=5050)
