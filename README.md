# ME32088 Mechatronics - Code Repository

This repository contains template code and resources for the **Mechatronics (ME32088)** teaching unit at the University of Bath.

## Overview

This codebase provides students with starter templates for common mechatronics tasks, including:
- Hardware communication via I2C bus protocols
- Computer vision with OpenCV and ArUco markers
- Network communication (TCP/UDP/MQTT)
- Circuit design reference schematics

## Repository Structure

### 📡 BusComs_Templates
I2C bus communication templates for sensor interfacing with Raspberry Pi.
- **i2cdetect** diagnostic guide
- Ready-to-use scripts for common sensors (LSM9DS1, MPU6050, NAU7802)
- See [BusComs_Templates/README.md](./BusComs_Templates/README.md) for detailed setup instructions

**Use case:** Reading IMU data, weight sensors, or other I2C peripherals on embedded systems.

### 🎥 CV_Templates
Computer vision and image processing templates using Python and OpenCV.
- OpenCV-based image capture and processing
- ArUco marker detection and tracking
- Camera calibration examples with pre-calibrated data
- Multiple templates for different complexity levels

**Use case:** Visual detection systems, marker-based tracking, camera-calibrated measurements.

### 🌐 Network_Templates
Network communication templates for distributed systems.
- **TCP**: Client-server communication examples
- **UDP**: Datagram-based communication
- **MQTT**: IoT publish-subscribe messaging (read-only and read-write variants)

**Use case:** Sending sensor data to remote servers, IoT dashboards, cloud platforms.

### 📋 Fritzing Schematics
Circuit design diagrams for reference and learning.

**Use case:** Understanding hardware wiring and component connections for your projects.

## Getting Started

1. **Choose a template** that matches your project requirements
2. **Review the documentation** in each folder's README (if available)
3. **Adapt the template code** for your specific hardware/sensors
4. **Test thoroughly** with your Raspberry Pi or development board

## Prerequisites

### Development Laptop
- **Python 3.8 or higher** installed on your system
  - Download from [python.org](https://www.python.org/downloads/) or use a package manager
  - Verify installation: `python3 --version`
- **Git** for cloning and version control
- **Terminal/Command Prompt**: Mac (Terminal) or Windows (PowerShell/CMD with OpenSSH)

### Raspberry Pi (provided for all students)
- Raspbian Bookworm 
- I2C enabled (for BusComs templates)
- `i2c-tools` installed
- Python libraries: OpenCV, numpy, paho-mqtt (as needed)
- SSH access for deploying code

## Development Environment Setup

### Step 0: Fork the Repository (Optional but Recommended)

A fork is your own GitHub copy of this repository, where you can make changes without changing the original.

1. Sign in to [GitHub](https://github.com/) and open <https://github.com/IGeorgilas/UoB_ME32088_Mechatronics>.
2. Select **Fork** in the upper-right corner.
3. Choose your GitHub account as the owner. Keep the default repository name and settings, then select **Create fork**.
4. Open your new fork and copy its URL. It should look like `https://github.com/YOUR-USERNAME/UoB_ME32088_Mechatronics`.

Skip this step if you only want to download the repository and do not need your own copy.


### Step 1: Clone the Repository

```bash
git clone https://github.com/IGeorgilas/UoB_ME32088_Mechatronics.git
cd UoB_ME32088_Mechatronics
```

### Step 2: Create a Virtual Environment

A virtual environment isolates project dependencies and prevents conflicts with other projects. In the following steps, replace `Mechatronics_env` with your preferred environment name if desired.

**On macOS/Linux:**
```bash
python3 -m venv Mechatronics_env
```

**On Windows (PowerShell):**
```powershell
python -m venv Mechatronics_env
```

**On Windows (Command Prompt):**
```cmd
python -m venv  Mechatronics_env
```

### Step 3: Activate the Virtual Environment

**On macOS/Linux:**
```bash
source Mechatronics_env/bin/activate
```

**On Windows (PowerShell):**
```powershell
Mechatronics_env\Scripts\Activate.ps1
```

> **Note:** If you get a permission error on Windows PowerShell, run:
> ```powershell
> Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
> ```

**On Windows (Command Prompt):**
```cmd
Mechatronics_env\Scripts\activate.bat
```

Once activated, your terminal prompt will show `(Mechatronics_env)` at the beginning.

### Step 4: Install Required Libraries

```bash
pip install --upgrade pip
pip install opencv-contrib-python numpy paho-mqtt
```

**Optional:** For Computer Vision advanced features:
```bash
pip install  matplotlib scipy
```

**Verify installation:**
```bash
python -c "import cv2; import numpy; import paho.mqtt.client; print('All packages installed successfully!')"
```

### Step 5: Deactivate Virtual Environment (when done)

```bash
deactivate
```

## Requirements Summary

The key Python packages used in this repository:

| Package | Purpose | Used in |
|---------|---------|---------|
| `opencv-contrib-python` | Computer vision and image processing | CV_Templates |
| `numpy` | Numerical computing and arrays | CV_Templates |
| `paho-mqtt` | MQTT client for IoT communication | Network_Templates |
| `matplotlib` | Data visualization (optional) | CV_Templates (advanced) |
| `scipy` | Scientific computing (optional) | CV_Templates (advanced) |

> **Note:** Some templates may only require a subset of these packages. Always check the specific template's documentation for its exact dependencies.

## Key Resources

- **Raspberry Pi Documentation**: Sensor configuration and enablement
- **OpenCV Docs**: Computer vision functions and techniques
- **MQTT Protocol**: IoT messaging standards

## Notes

- Windows users: Ensure OpenSSH is installed for `scp`/`ssh` commands
- File encoding: If you encounter issues, check that scripts use Unix line endings
- Hardware addressing: Verify sensor I2C addresses match your configuration

## Support

For issues or questions about template usage, refer to the specific folder's documentation or contact the module leader.

---

**Module:** ME32088 - Mechatronics  
**Institution:** University of Bath
**Author:** Ioannis Georgilas
**Copyright:** © 2020-2026 Ioannis Georgilsa. University of Bath. All rights reserved.