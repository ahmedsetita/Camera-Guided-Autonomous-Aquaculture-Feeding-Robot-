<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&logoColor=white" alt="ROS 2 Humble"/>
  <img src="https://img.shields.io/badge/Gazebo-Classic%2011-orange?logo=gazebo&logoColor=white" alt="Gazebo Classic 11"/>
  <img src="https://img.shields.io/badge/ESP32-C++-green?logo=espressif&logoColor=white" alt="ESP32 Firmware"/>
  <img src="https://img.shields.io/badge/OpenCV-4.x-red?logo=opencv&logoColor=white" alt="OpenCV"/>
  <img src="https://img.shields.io/badge/Edge%20AI-YOLOv8n%20%2F%20YOLO11n%20Ready-purple" alt="YOLO Edge Ready"/>
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%204-c51a4a?logo=raspberrypi&logoColor=white" alt="Raspberry Pi 4"/>
  <img src="https://img.shields.io/badge/Sensors-4x%20Ultrasonic%20%2B%205x%20Water-teal" alt="4x Ultrasonic + 5x Water Sensors"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow" alt="MIT License"/>
</p>

# 🐟 Autonomous Aquaculture Feeding & Water-Quality Robot

> **A low-cost, camera-driven autonomous surface robot with 4-side ultrasonic obstacle avoidance, ESP32 industrial sensor filtering, closed-loop precision dosing, and a live Web Monitoring & Automation Dashboard.**

Feed accounts for **60–70% of operating costs** in commercial fish farms. Over-feeding wastes pellets, spikes ammonia/turbidity, crashes **Dissolved Oxygen (DO)**, and risks mass fish mortality; under-feeding slows harvest growth. This robot drives itself **purely via onboard camera vision** toward floating colored zone buoys, protects its hull with **4-side ultrasonic sensors**, evaluates live water quality and fish feeding activity at each buoy, and automatically controls feed dosing and pond aeration.

---

## 🎬 Demo Video

<p align="center">
  <a href="https://youtu.be/iXm9WUoXbpE">
    <img src="https://img.youtube.com/vi/iXm9WUoXbpE/maxresdefault.jpg" alt="Autonomous Aquaculture Robot Simulation Demo" width="100%"/>
  </a>
  <br/>
  <em>▶️ <b>Click above to watch the autonomous simulation demo on YouTube</b></em>
</p>

---

## 🎯 Core Engineering Advantages

| Design Choice | Why It Matters in Real Fish Ponds |
|---------------|-----------------------------------|
| **Pure Camera-Driven Navigation** | The forward RGB camera directly drives robot motion (`cmd_vel`) via visual servoing toward colored buoys — avoiding expensive **LiDAR** sensors (whose beams scatter on open water) and working without pre-programmed coordinates. |
| **4-Side Ultrasonic Avoidance Ring** | Front, Left, Right, and Rear ultrasonic range sensors (~$2 each) let the robot cruise randomly around banks, docks, and pond obstacles between buoys without collisions. |
| **Biological + Water-Safe Gating** | Checks live `DO`, `pH`, `Temperature`, `Turbidity`, and visual fish agitation before opening the dispenser valve — stopping wasted feed in hypoxic zones. |
| **Closed-Loop Action & Live Dashboard** | Dispenses exact gram doses via HX711 weight feedback, keeps the feeder locked when water parameters fail safe limits, switches **Aerators (`ON · AUTO`)**, and logs every inspection live to the Web UI. |

---

## 📐 System Architecture

```text
┌──────────────────────────────────────────────────────────────────────────────┐
│                 ROS 2 COMPUTE LAYER (PC Simulation OR Raspberry Pi 4)        │
│                                                                              │
│  ┌──────────────────────┐    /vision/buoy_detections     ┌─────────────────┐ │
│  │  fish_vision_node.py │─── /vision/feeding_demand ────▶│ autonomous_     │ │
│  │  (Dual OpenCV 4.x    │                                │ brain_node.py   │ │
│  │   CPU Pipeline)      │◀── /aqua/target_buoy ──────────│ (5-State FSM +  │ │
│  └──────────┬───────────┘                                │  4x Ultrasonic) │ │
│             │ /vision/debug_image                        └────────┬────────┘ │
│             ▼                                                     │          │
│  ┌──────────────────────┐    /aqua/decision_report                │          │
│  │ aqua_web_dashboard   │◀────────────────────────────────────────┤          │
│  │ _node.py (:8080)     │─── /aqua/safe_limits & dispatch_cmd ───▶│          │
│  │ Live Water Quality   │                                         │          │
│  │ UI + /stream.mjpg    │    /cmd_vel       /feeder_valve_cmd     │          │
│  └──────────────────────┘       │                  │              │          │
│                                 ▼                  ▼              │          │
│  ┌──────────────────────┐   Thrusters     ┌───────────────────────────┐      │
│  │ pond_rviz_visualizer │   / Gazebo      │ esp32_sim_node.py (Sim)   │      │
│  │ (3D Pond, Buoys, US) │   planar_move   │ esp32_serial_bridge (HW)  │      │
│  └──────────────────────┘                 └────────────┬──────────────┘      │
└────────────────────────────────────────────────────────┼─────────────────────┘
                                                         │ USB UART (115200 bps, JSON @ 20 Hz)
┌────────────────────────────────────────────────────────┼─────────────────────┐
│                        ▼   ESP32 REAL-TIME MCU LAYER   │                     │
│  ┌───────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┴──┐  ┌─────────────┐ │
│  │ HX711     │  │ pH BNC   │  │ DO Probe │  │ Turbidity   │  │ DS18B20     │ │
│  │ GPIO 16/4 │  │ GPIO 34  │  │ GPIO 35  │  │ GPIO 32     │  │ GPIO 27     │ │
│  └───────────┘  └──────────┘  └──────────┘  └─────────────┘  └─────────────┘ │
│  30x Oversampling → 20% Trimmed Mean → 1D Kalman → EMA → Servo (GPIO 18)     │
└──────────────────────────────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────────────────────────────┐
│                     PERCEPTION & OBSTACLE RING (ROBOT HULL)                  │
│   📷 Forward Camera (640×480, 90° FOV)    📡 4× Ultrasonic (Front/L/R/Rear)  │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 🧠 How Camera Navigation Works (5-State FSM)

Motion is controlled **purely by the camera and 4 ultrasonic sensors** — there are no fixed waypoints or external positioning systems:

```text
          ┌──────────────[ Buoy seen & off cooldown ]──────────────┐
          ▼                                                        │
   ┌──────────────┐                                         ┌──────┴──────────┐
   │RANDOM_SEARCH │◀────[ 2.3s clear turn + 32s cooldown ]──│  APPROACH_BUOY  │
   │(4x US Avoid) │                                         │(Camera Servoing)│
   └──────────────┘                                         └──────┬──────────┘
          ▲                                                        │
          │                                             [ Area ≥ threshold / close ]
          │                                                        ▼
   ┌──────────────┐     [ Unsafe water OR low demand ]      ┌─────────────────┐
   │ DEPART_BUOY  │◀────────────────────────────────────────│ INSPECT_AND_    │
   │              │     ┌──────────────┐                    │ DECIDE (2.5s)   │
   └──────▲───────┘     │EXECUTE_FEEDING◀─[ Safe + Demand ]─┴─────────────────┘
          └─────────────┤(HX711 Dosing)│
        [ Dose reached ]└──────────────┘
```

1. **`RANDOM_SEARCH`**: Cruises smoothly across open water while all **4 ultrasonic sensors** (`/ultrasonic/front`, `/ultrasonic/left`, `/ultrasonic/right`, `/ultrasonic/rear`, 0.08–4.50 m range) steer around pond banks, docks, and mid-pond obstacles until the camera sees an active colored buoy.
2. **`APPROACH_BUOY`**: Camera visual servoing locks onto the target buoy (`GREEN`, `YELLOW`, `RED`, or `MAGENTA`), tracks horizontal image offset (`error_x`) to steer directly toward the ball, and checks front/side ultrasonics for safe clearance.
3. **`INSPECT_AND_DECIDE`**: Holds beside the buoy for `2.5s`, reads filtered ESP32 water sensors against live safe limits (`DO ≥ 5.0 mg/L`, `pH 6.5–9.0`, `Temp 25–30 °C`, `Turbidity < 40%`), checks visual fish demand (`0–100%`), and publishes a structured decision report to `/aqua/decision_report`.
4. **`EXECUTE_FEEDING` vs. Safety Lockout**:
   - **Water Safe & Demand ≥ 35%**: Dynamically calculates pellet dose (`~180–250 g` based on fish feeding demand), opens `/feeder_valve_cmd`, tracks closed-loop HX711 hopper weight until the exact dose is dispensed, then closes the valve.
   - **Water Anomaly Detected** (e.g., `RED` zone): Keeps the feeder valve **CLOSED**, identifies **exactly which parameter failed and by how much**, switches **`Aerators: ON · AUTO`**, and triggers an alert banner on the web dashboard.
5. **`DEPART_BUOY`**: Applies a `32s` cooldown to the inspected buoy, performs an ultrasonic-safe turn away from the buoy, and resumes `RANDOM_SEARCH` for the next buoy.

---

## 👁️ Computer Vision Pipeline & YOLO Model Strategy (Edge Deployment)

### 1. Active Runtime Vision Pipeline (`scripts/fish_vision_node.py`)
Currently, [`fish_vision_node.py`](scripts/fish_vision_node.py) runs a **two-stage CPU-only OpenCV 4.x pipeline** engineered specifically so it runs smoothly alongside ROS 2 and the Web Server on Raspberry Pi without needing a GPU:
- **Stage A — Colored Buoy Visual Guidance (`_detect_buoys`)**: Converts `640×480` frames to HSV, segments `GREEN`, `YELLOW`, `RED`, and `MAGENTA` buoys, applies `5×5` elliptical morphological open/close filtering, verifies spherical shape (`area ≥ 140 px`, aspect ratio `0.40–2.30`, fill ratio `≥ 0.40`), and outputs horizontal steering error (`error_x`) + proximity (`area_ratio`).
- **Stage B — Fish Feeding Demand Scoring (0–100%)**: Combines amber-orange fish contour detection, feeding-pad/anoxic floor signatures, and Gaussian-smoothed temporal frame differencing (`cv2.absdiff`) smoothed via Exponential Moving Average (`α = 0.35`).
- **Measured Footprint**: **~155 MB RAM** and **< 15 ms CPU latency** per frame.

### 2. Phase 2 Edge AI Upgrade — YOLO Variant Selection for Raspberry Pi 4
For outdoor commercial scaling (challenging water surface glare, partially occluded buoys, multi-species fish counting, or floating debris detection), the recommended architecture for **Raspberry Pi 4 (ARM64 Cortex-A72 CPU)** is quantized **Nano YOLO**:

| Model / Runtime Option | Params | Input Size | Footprint on Pi 4 | Latency (Pi 4 CPU) | Engineering Fit |
|------------------------|--------|------------|-------------------|--------------------|-----------------|
| **Current Baseline: OpenCV HSV + Contours + Motion Diff** | N/A | 640×480 | **~155 MB** (Measured) | **< 15 ms (15–30 FPS)** | ✅ Running now — deterministic visual servoing with zero GPU overhead |
| **Phase 2 Recommended: YOLOv8n / YOLO11n (`NCNN` or `ONNX INT8`)** | ~2.6M–3.2M | 320×320 / 416×416 | **~220–350 MB** (Est.) | **~60–120 ms (8–16 FPS)** | ✅ Best neural detector for ARM64 CPU on real ponds |
| Heavy PyTorch YOLOv8s / YOLOv8m | 11.2M–25.9M | 640×640 | **~1.1–1.8 GB+** | 400–1200+ ms (< 2 FPS) | ❌ Too heavy for real-time Raspberry Pi control |

> Because perception is decoupled via ROS 2 topics (`/vision/buoy_detections`, `/vision/feeding_demand`), swapping OpenCV for quantized **YOLO11n / YOLOv8n (`NCNN`)** requires zero changes to the FSM controller, ESP32 bridge, or Web Dashboard.

---

## 🌐 Live Water Quality Web Dashboard

Implemented in `scripts/aqua_web_dashboard_node.py` using Python's built-in `ThreadingHTTPServer` (zero external frontend build steps — runs directly on Raspberry Pi or simulation PC):

- **URL:** [`http://localhost:8080`](http://localhost:8080) (opens automatically on launch)
- **Live Metric Cards & Sparklines:** Dissolved Oxygen (`mg/L`), `pH`, Temperature (`°C`), Turbidity (`%`), and Hopper Weight (`g`).
- **Decision & Action Card:** Explains what happened at the latest buoy (what parameter was out of range if skipped, or exact grams dispensed when valve opened).
- **Automation Card:** Live status for **Aerators** (`OFF · AUTO` / `ON · AUTO`), **Feeder** (`Running` / `Paused (Safety)`), and **Phone Alerts**.
- **Interactive Controls:**
  - **View history** (`GET /api/history`) — inspection timeline with timestamped readings and actions.
  - **Edit limits** (`POST /api/limits`) — per-species safe threshold presets (`Nile Tilapia`, `African Catfish`, `European Sea Bass`) synced live to `/aqua/safe_limits`.
  - **📷 Robot Vision & 4x Ultrasonics Modal** (`/stream.mjpg` + `/api/command`) — live annotated camera stream, 4-side ultrasonic clearance gauges, and manual commands (`AUTO`, `SEEK_<COLOR>`, `Valve`, `Refill`, `Pause`).

---

## 📊 Measured RAM & Compute Footprint

Measured via `ps` (RSS) on Ubuntu 22.04 + ROS 2 Humble in live headless autonomous simulation (`Nov 13, 2026`):

### 1. Simulation Mode (PC — Gazebo Classic + Headless Stack)
| Process | Measured RSS RAM | Role |
|---------|------------------|------|
| `gzserver` | ~382 MB | Gazebo 11 physics + 4× ultrasonic ray sensors + RGB camera |
| `aqua_web_dashboard_node.py` | ~155 MB | Local HTTP server + MJPEG streamer + telemetry history |
| `fish_vision_node.py` | ~155 MB | OpenCV 4.x dual buoy + fish feeding demand pipeline |
| `pond_rviz_visualizer_node.py` | ~51 MB | 2D OccupancyGrid + 3D buoys, obstacles & ultrasonic markers |
| `autonomous_brain_node.py` | ~50 MB | 5-State FSM, visual servoing & ultrasonic avoidance |
| `esp32_sim_node.py` | ~50 MB | Spatial water sensor simulator + 3-stage filter |
| `robot_state_publisher` + `static_tf` | ~31 MB | URDF transforms (`map → odom → base_footprint → sensors`) |
| **Total Headless Simulation Stack** | **~874 MB** | *Add ~500–600 MB extra when `gui:=true rviz:=true` desktop windows are open* |

### 2. Real Hardware Deployment (Raspberry Pi 4, 4 GB RAM) + ESP32
On the physical boat (`use_real_hardware:=true`), `gzserver`, `gzclient`, `rviz2`, and `esp32_sim_node` do not run:

| Process on Raspberry Pi 4 | Footprint | Notes |
|---------------------------|-----------|-------|
| `aqua_web_dashboard_node.py` | ~155 MB (Measured node) | Hosts Web UI over Wi-Fi hotspot / LAN |
| `fish_vision_node.py` (OpenCV) | ~155 MB (Measured node) | Sub-15 ms CPU vision |
| `autonomous_brain_node.py` | ~50 MB (Measured node) | Camera servoing + ultrasonic FSM |
| `esp32_serial_bridge_node.py` | ~50 MB (Est. matching rclpy) | 115200 baud UART JSON bridge to ESP32 |
| `robot_state_publisher` + TF | ~31 MB (Measured nodes) | Robot TF tree |
| **Total ROS 2 App Stack (OpenCV)** | **~441 MB** | **Leaves > 3.0 GB free RAM** on Pi 4 (4 GB) incl. OS |
| **With Phase 2 YOLO11n / YOLOv8n (NCNN)** | **~660–790 MB** | Fits comfortably inside Pi 4 (4 GB) with > 2.5 GB headroom |

---

## 🔧 Hardware Architecture & Prototype BOM

### ESP32-WROOM-32 Wiring (`firmware/esp32_aquaculture_firmware.ino`)
The ESP32 handles deterministic **20 Hz** sampling and actuator safety with **30× Oversampling → 20% Trimmed Mean → 1D Kalman Filter → EMA**:

| ESP32 Pin | Component | Interface | Purpose |
|-----------|-----------|-----------|---------|
| `GPIO 16 / 4` | HX711 + 5 kg Load Cell | Digital (`DOUT / SCK`) | Closed-loop hopper pellet weight & dose verification |
| `GPIO 34` | Analog pH Probe (BNC) | `ADC1_CH6` (12-bit, 11 dB) | Pond acidity (`0–14 pH`) |
| `GPIO 35` | Analog Dissolved Oxygen Probe | `ADC1_CH7` (12-bit, 11 dB) | Oxygen safety (`mg/L`) |
| `GPIO 32` | Analog Turbidity Sensor | `ADC1_CH4` (12-bit, 11 dB) | Suspended solids / clarity (`NTU / %`) |
| `GPIO 27` | DS18B20 Waterproof Probe | 1-Wire (4.7 kΩ pull-up) | Water temperature (`°C`) + compensation |
| `GPIO 18 / 19` | Dispenser Servo + Relay | LEDC PWM (50 Hz) + Digital | Auger / gate valve actuation (`1000 µs` closed / `1900 µs` open) |
| `GPIO 2` | Status LED | Digital | Valve & telemetry indicator |

### Prototype Hardware Components
| Subsystem | Specification | Purpose |
|-----------|--------------|---------|
| **Main Compute** | Raspberry Pi 4 (4 GB) | Runs ROS 2 Humble, vision, 5-State FSM & Web Dashboard |
| **Sensor MCU** | ESP32-WROOM-32 DevKit | Real-time 20 Hz sensor DSP filtering & valve PWM/relay control |
| **Camera** | Waterproof USB Camera (640×480 @ 30 FPS) | Colored buoy visual guidance & fish feeding activity |
| **Obstacle Sensors** | 4× Waterproof Ultrasonics (`JSN-SR04T` w/ voltage divider) | Front, Left, Right & Rear collision avoidance |
| **Water Sensors** | Analog DO, pH, Turbidity + DS18B20 | Continuous water quality monitoring |
| **Dosing** | 5 kg Hopper + HX711 Load Cell + Waterproof Servo | Precision gram-accurate feeding |
| **Propulsion & Power** | 2× 12V Thrusters + ESCs + 12V LiPo (≥5000 mAh) + 5V UBEC | Differential boat drive & regulated compute supply |

---

## 📁 Project Structure & ROS 2 Topics

```text
aquaculture_autonomous_robot/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── urdf/
│   └── aqua_robot.urdf.xacro           # Boat chassis, hopper, valve, RGB camera, IMU, 4x ultrasonics
├── worlds/
│   └── fish_farm.world                 # 14x14 m pond, 4 colored floating buoys, fish zones, obstacles
├── launch/
│   └── full_simulation.launch.py       # Unified launch (Gazebo, RViz2, Vision, Brain, Web UI, ESP32)
├── scripts/
│   ├── fish_vision_node.py             # Dual OpenCV buoy guidance + fish demand pipeline
│   ├── autonomous_brain_node.py        # 5-State FSM (Camera servoing + 4x ultrasonic avoidance)
│   ├── aqua_web_dashboard_node.py      # Live Water Quality Web Dashboard (:8080) + MJPEG stream
│   ├── esp32_sim_node.py               # Spatial zone sensor simulator + 3-stage industrial filter
│   ├── esp32_serial_bridge_node.py     # Real ESP32 USB-UART JSON bridge (115200 baud)
│   ├── pond_rviz_visualizer_node.py    # 3D RViz buoys, obstacles, ultrasonic rays, fish & HUD
│   └── aqua_control_gui.py             # Optional desktop Tkinter telemetry GUI
├── firmware/
│   └── esp32_aquaculture_firmware.ino  # Production non-blocking ESP32 C++ firmware
└── rviz/
    └── aqua_robot.rviz                 # Pre-configured RViz2 layout
```

### Key ROS 2 Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Front/downward 640×480 RGB stream (90° FOV) |
| `/ultrasonic/{front,left,right,rear}` | `sensor_msgs/Range` | 4-side ultrasound obstacle distances (`0.08–4.50 m` @ `15 Hz`) |
| `/vision/buoy_detections` | `std_msgs/String` | JSON detections for `GREEN`, `YELLOW`, `RED`, `MAGENTA` buoys |
| `/vision/feeding_demand` | `std_msgs/Float32` | Fish feeding demand score (`0–100%`) |
| `/vision/debug_image` | `sensor_msgs/Image` | Annotated camera view with target reticles, fish boxes & HUD |
| `/esp32/sensors_filtered` | `std_msgs/String` | JSON packet: `weight`, `ph`, `do`, `turbidity`, `turbidity_pct`, `temp`, `valve` |
| `/aqua/decision_report` | `std_msgs/String` | Structured inspection report, missing-parameter alerts & dose summary |
| `/aqua/safe_limits` | `std_msgs/String` | Live species limits synced between Web Dashboard & Autonomous Brain |
| `/aqua/dispatch_cmd` | `std_msgs/String` | Manual overrides (`AUTO`, `SEEK_<COLOR>`, `PAUSE`, `RESUME`) |
| `/feeder_valve_cmd` / `/feeder_refill_cmd` | `std_msgs/Bool` | Feeder gate valve control & hopper refill trigger |
| `/cmd_vel` | `geometry_msgs/Twist` | Thruster velocity commands driven by camera + ultrasonic avoidance |

---

## 🚀 Installation & Usage

### 1. Prerequisites (Ubuntu 22.04 + ROS 2 Humble)
```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-cv-bridge ros-humble-xacro ros-humble-rviz2 \
  python3-opencv python3-numpy python3-serial
```

### 2. Clone & Build
> If Miniconda/Anaconda is installed, clear it from `PATH` before building so `colcon` links against ROS 2's system `/usr/bin/python3`.

```bash
mkdir -p ~/autonomous_ws/src && cd ~/autonomous_ws/src
git clone https://github.com/ahmedsetita/ROS-2-Aquaculture-Autonomous-Robot-Package.git aquaculture_autonomous_robot

cd ~/autonomous_ws
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
colcon build --packages-select aquaculture_autonomous_robot
source install/setup.bash
```

### 3. Launch Full Simulation + Live Web Dashboard
```bash
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source ~/autonomous_ws/install/setup.bash
ros2 launch aquaculture_autonomous_robot full_simulation.launch.py
```
- Opens **Gazebo Classic**, **RViz2**, and the **Water Quality Web Dashboard** at **[`http://localhost:8080`](http://localhost:8080)**.

#### Launch Variations
```bash
# Headless simulation + Web Dashboard only (low RAM)
ros2 launch aquaculture_autonomous_robot full_simulation.launch.py gui:=false rviz:=false

# Real Hardware Mode on Raspberry Pi 4 + Physical ESP32
ros2 launch aquaculture_autonomous_robot full_simulation.launch.py \
  use_real_hardware:=true serial_port:=/dev/ttyUSB0 gui:=false rviz:=false

# Include optional Tkinter debug window alongside Web Dashboard
ros2 launch aquaculture_autonomous_robot full_simulation.launch.py control_gui:=true
```

---

## 🛣️ Roadmap

- [x] Pure camera visual servoing driving robot motion toward floating colored buoys (`GREEN`, `YELLOW`, `RED`, `MAGENTA`)
- [x] 4-Side ultrasonic obstacle avoidance ring (`Front / Left / Right / Rear`) during random pond search
- [x] 5-State Autonomous FSM with demand-scaled feed dosing & unsafe-water valve lockout
- [x] Live Water Quality Web Dashboard (`:8080`) with sparklines, history log, editable species limits & MJPEG vision stream
- [x] Production ESP32 C++ firmware (`30x` oversampling + trimmed mean + Kalman + EMA + JSON UART)
- [ ] Phase 2 Edge AI: Quantized **YOLO11n / YOLOv8n (`NCNN`)** integration on Raspberry Pi 4/5 for outdoor glare & biomass estimation
- [ ] Physical thruster ESC PWM driver node + solar MPPT battery telemetry
- [ ] Multi-pond coordination & mobile alert webhooks

---

## 👨‍💻 Author

**Ahmed Setita**
- GitHub: [@ahmedsetita](https://github.com/ahmedsetita)
- Email: `ahmedsetita599@gmail.com`
- Repository: [ROS-2-Aquaculture-Autonomous-Robot-Package](https://github.com/ahmedsetita/ROS-2-Aquaculture-Autonomous-Robot-Package)

---

<p align="center">
  <strong>MIT License — Built for Sustainable Precision Aquaculture (2026)</strong>
</p>
