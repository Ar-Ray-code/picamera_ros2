# picamera_ros2

PiCamera wrapper using libcamera (RasPi-CSI2)

<br>

## Depends (Development environment)

- Raspberry Pi Bullseye (32bit)
- libcamera
- [ROS2-Galactic](https://github.com/Ar-Ray-code/rpi-bullseye-ros2)
- OpenCV4

```bash
sudo apt install build-essential cmake git libcamera-dev libopencv-dev
```

<br>

## Usage

```bash
source /opt/ros/galactic/setup.bash
git clone https://github.com/Ar-Ray-code/picamera_ros2.git ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
```

<br>

## Context (by kbarni)

In Raspbian Bullseye, the Raspberry Pi camera framework was completely rebased from MMAL to the libcamera library - thus breaking most of the previous camera dependencies.

Raspbian comes with the handy libcamera-apps package that duplicates the old raspistill and raspivid applications, with some added functionnality, like the possibility of adding postprocessing routines to the capturing process.

However this is still limited, as it doesn't allow full integration of the camera in your software.

LCCV aims to provide a simple to use wrapper library that allows you to access the camera from a C++ program and capture images in cv::Mat format.

<br>

## License

BSD 2-Clause License (Raspberry Pi)

## Reference

- [LCCV](https://github.com/kbarni/LCCV) by kbarni
- [libcamera-apps](https://github.com/raspberrypi/libcamera-apps) by raspberrypi