# pinax_camera_model_jazzy

Pinax (underwater refractive) camera model for ROS2 Jazzy. Used for stereo image correction on the ROV vision pipeline.

## Build

### Prerequisites
```bash
sudo apt install ros-jazzy-desktop ros-jazzy-image-transport
sudo apt install python3-rosdep colcon-common-extensions
```

### Install dependencies
```bash
cd ~/Project/ROV_ws
rosdep install --from-paths \
  src/rov_perception/pinax_camera_model_jazzy \
  --ignore-src -r -y
```

### Build
```bash
# Builds jir_image_remapper and all its dependencies in correct order
colcon build --packages-up-to jir_image_remapper
source install/setup.zsh
```
