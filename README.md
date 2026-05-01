# LNX InfraBot

An autonomous soccer robot for **RoboCup Junior** (Infrared/IR version) that seamlessly integrates hardware control and computer vision. The system uses a Raspberry Pi for advanced vision processing and control logic, while a Teensy microcontroller manages real-time motor and sensor operations.

---

## Project Overview

**LNX-InfraBot** is a competition-ready robot featuring:

- **RoboCup Junior Compliance**: Designed for the Infrared competition category
- **Advanced Computer Vision**: Real-time image processing with OpenCV and NumPy for ball and goal detection
- **Robust Hardware Control**: Motor and sensor integration via Teensy microcontroller
- **Web Dashboard Integration**: Seamless connection with [lnx-dashboard](https://github.com/anton2026gamca/lnx-dashboard) for calibration, remote monitoring, and control
- **Reliable Serial Communication**: High-speed protocol between Raspberry Pi and Teensy

---

## Hardware Components

### Raspberry Pi
- Runs main control software (Python)
- Interfaces with PiCamera2 for vision input
- Hosts FastAPI web server
- Communicates with Teensy via serial connection

### Teensy Microcontroller
- Real-time motor and sensor control
- Manual control switches and status LEDs
- High-speed serial communication (84 kbaud, targeting 120 messages/second)

### 3D Printed Components
- Custom chassis and parts (see `hardware/3D design/`)

---

## Project Structure

```
lnx-infrabot/
├── raspberrypi/             # Raspberry Pi software
│   ├── main.py              # Entry point
│   ├── requirements.txt     # Python dependencies
│   ├── robot.service        # Systemd service file
│   └── robot/               # Main robot package
│       ├── config.py        # Configuration management
│       ├── robot.py         # Core robot control
│       ├── api/             # FastAPI web server
│       ├── hardware/        # Hardware interfaces
│       ├── vision/          # Computer vision processing
│       ├── logic/           # Control logic and algorithms
│       ├── calibration/     # Calibration utilities
│       ├── multiprocessing/ # Concurrent process management
│       └── utils/           # Utility functions
├── teensy/                  # Teensy firmware
│   ├── teensy.ino           # Main Teensy sketch
│   ├── sketch.yaml          # Arduino sketch configuration
│   ├── structures.txt       # Data structure definitions
│   └── test_programs/       # Testing utilities
├── hardware/                # Hardware designs and documentation
│   └── 3D design/           # 3D printable robot parts
├── CONTRIBUTING.md          # Contribution guidelines
└── README.md                # This file
```

---

## Installation

### Raspberry Pi Setup

- **Prerequisites:**
    - **Raspberry Pi 5** with **Raspberry Pi OS Lite** installed

1. **Configure UART and Camera**  
   Edit `/boot/firmware/config.txt` using a text editor like `nano`:
   ```sh
   sudo nano /boot/firmware/config.txt
   ```
   Add or update the following lines:
   ```txt
   # UART 
   dtoverlay=uart0-pi5

   # Camera
   camera_auto_detect=0
   dtoverlay=imx708
   ```
   After editing, press `Ctrl+X` and then `Y` to exit and save.
   Disable the serial console with `sudo raspi-config` if needed.

   ```bash
   sudo reboot
   ```

   For more info about the camera, see the [official documentation](https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/12MP-IMX708/#software-configuration)

2. **Add yourself to the `bluetooth` group**
   ```bash
   sudo usermod -aG bluetooth $USER
   ```

   Relogin or reboot
   ```bash
   sudo reboot
   ```

3. **(Optional) Enable Passwordless Bluetooth Discoverability**

   Some Bluetooth operations (like making the device discoverable) require root privileges. To allow the robot software to set Bluetooth discoverable mode without prompting for a password, you can configure passwordless sudo only for the `hciconfig` command:

   1. Open the sudoers file for editing:
      ```bash
      sudo visudo
      ```
   2. Add the following line at the end (replace `pi` with your username if different):
      ```
      pi ALL=NOPASSWD: /usr/bin/hciconfig
      ```
   3. Save and exit. Now, the robot software can run `sudo hciconfig ...` without a password prompt.

4. **Install Dependencies**
   ```bash
   sudo apt install git python3.13-dev libcap-dev libgl1 libcamera-apps python3-libcamera
   sudo apt install python3-picamera2 --no-install-recommends
   ```

5. **Clone the Repository**
   ```bash
   git clone --no-checkout https://github.com/anton2026gamca/lnx-infrabot
   cd lnx-infrabot
   git sparse-checkout init --cone
   git sparse-checkout set raspberrypi
   git checkout main
   ```

6. **Set Up Python Virtual Environment**
   ```bash
   cd raspberrypi
   python3 -m venv .venv
   source .venv/bin/activate
   ```

7. **Install Python Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

8. **Set Environment Variables**
   Create a `.env` file:
   ```
   AUTH_TOKEN=your-secure-token     # Defaults to `ooops` if not set
   ```

9. **Enable Auto-Start**
   ```bash
   sudo ln -s "$(pwd)/robot.service" /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable robot
   sudo systemctl start robot
   ```

### Teensy Setup

1. Open `teensy/teensy.ino` in Arduino IDE
2. Install library dependency: **teensy4_i2c** by Richard Gemmell
3. Select Board: **Teensy 4.1**
4. Upload the sketch to your Teensy board

---

## Usage

### Web API & Dashboard

The robot exposes a FastAPI web server with WebSocket support for real-time communication.

Connect using the [lnx-dashboard](https://github.com/anton2026gamca/lnx-dashboard) for:
- Live camera feed with vision overlays
- Robot telemetry (motion, sensor data)
- Manual and autonomous control

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md)

---

## Related Projects

- [lnx-dashboard](https://github.com/anton2026gamca/lnx-dashboard) — Web-based monitoring and control dashboard for LNX-InfraBot

---

## RoboCup Junior Information

LNX-InfraBot is designed for the **RoboCup Junior Soccer - Infrared (IR) League**. Learn more:
- [Official RoboCup Website](https://www.robocup.org/)

---

## License

See [LICENSE](LICENSE)
