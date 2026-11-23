# Computer Vision Templates

This folder contains Python templates and utilities for computer vision applications, with a focus on camera calibration and ArUco marker detection.

## Files

- **CV_Python_template.py** - Basic computer vision template for image processing tasks
- **CV_Python_template_with_resolutions.py** - Extended template with multiple resolution support
- **CV_Python_Aruco_4_10.py** - ArUco marker detection and pose estimation (works with OpenCV 4.7+ and OpenCV 5)
- **aruco_calibration_IGversion.ipynb** - Jupyter notebook for camera calibration using ArUco markers
- **Sample_Calibration.npz** - Pre-computed camera calibration parameters

## Usage

These templates provide a foundation for computer vision projects including:
- Real-time image capture and processing
- Camera calibration using ArUco markers
- Marker detection and pose estimation
- Frame rate measurements and performance monitoring

## Requirements

Typical dependencies include:
- OpenCV (`pip install opencv-contrib-python` for ArUco support)
- NumPy and Matplotlib for numerical operations and plotting 
(`pip install numpy matplotlib`)
- Jupyter (`pip install pillow ipykernel` for notebook support)
