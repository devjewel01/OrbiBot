# RealSense D435 Setup Guide for OrbiBot

## Common Issues and Solutions

### 🔧 **Issue: "Depth stream start failure"**

This is a common USB bandwidth issue on Raspberry Pi. Here are the solutions:

### **Solution 1: Use Optimized Launch File**

```bash
# Use the reliable launch configuration
ros2 launch orbibot_navigation realsense_reliable.launch.py

# This uses optimized settings:
# - Lower resolution: 424x240 instead of 640x480  
# - Lower FPS: 15 instead of 30
# - Reduced laser power for stability
```

### **Solution 2: Manual Resolution Control**

```bash
# Start with minimal settings
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=424x240x15 \
    rgb_camera.profile:=424x240x15 \
    enable_gyro:=false \
    enable_accel:=false

# If stable, gradually increase resolution
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=640x480x15 \
    rgb_camera.profile:=640x480x15
```

### **Solution 3: USB Optimization**

```bash
# Check USB version and power
lsusb -t | grep -A3 "Intel"

# Add USB mount options (if needed)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b07", MODE="0666"' | sudo tee /etc/udev/rules.d/99-realsense.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### **Solution 4: System Optimization**

```bash
# Increase USB memory (add to /boot/firmware/config.txt)
echo "dwc_otg.fiq_enable=1" | sudo tee -a /boot/firmware/config.txt
echo "dwc_otg.fiq_fsm_enable=1" | sudo tee -a /boot/firmware/config.txt  
echo "dwc_otg.nak_holdoff=1" | sudo tee -a /boot/firmware/config.txt

# Restart after changes
sudo reboot
```

## 📊 **Verification Commands**

### Check Camera Detection
```bash
# List USB devices
lsusb | grep Intel

# Check RealSense detection
rs-enumerate-devices

# List camera info
rs-sensor-control
```

### Verify ROS Topics
```bash
# Check topics (after launching camera)
ros2 topic list | grep camera

# Expected topics:
# /camera/camera_info
# /camera/color/image_raw
# /camera/depth/image_rect_raw
# /camera/aligned_depth_to_color/image_raw
# /camera/depth/color/points

# Test data flow
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_rect_raw
```

### Monitor Performance
```bash
# Check CPU usage
htop

# Monitor USB bandwidth
sudo iotop

# Check camera temperatures
rs-sensor-control
```

## 🎛️ **Recommended Settings for OrbiBot**

### **Navigation Mode (Balanced)**
```bash
ros2 launch orbibot_navigation realsense_reliable.launch.py
```

### **High Performance Mode** (if stable)
```bash
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    pointcloud.enable:=true \
    align_depth.enable:=true
```

### **Low Power Mode** (battery operation)
```bash
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=424x240x10 \
    rgb_camera.profile:=424x240x10 \
    enable_gyro:=false \
    enable_accel:=false \
    depth_module.laser_power:=100
```

## 🔄 **Integration with OrbiBot Navigation**

### **Step 1: Test Camera**
```bash
# Start camera
ros2 launch orbibot_navigation realsense_reliable.launch.py

# Verify in new terminal
ros2 topic echo /camera/color/image_raw --once
```

### **Step 2: Start Enhanced Navigation**
```bash
# Launch visual-enhanced navigation
ros2 launch orbibot_navigation realsense_navigation.launch.py \
    enable_visual_odometry:=true \
    enable_depth_obstacles:=true
```

### **Step 3: Monitor Integration**
```bash
# Check visual odometry
ros2 topic echo /visual_odometry/odom

# Check depth obstacles
ros2 topic echo /depth_scan

# Check enhanced localization
ros2 topic echo /orbibot/odometry/enhanced
```

## ⚠️ **Troubleshooting**

### **Camera Not Detected**
```bash
# Check physical connection
lsusb | grep Intel

# Check permissions
ls -la /dev/video*

# Reset USB device
sudo usb_modeswitch -v 8086 -p 0b07 --reset-usb
```

### **Poor Depth Quality**
```bash
# Check IR emitter
rs-sensor-control  # Ensure emitter is enabled

# Adjust lighting conditions
# - Avoid direct sunlight
# - Ensure adequate lighting
# - Remove reflective surfaces nearby
```

### **High CPU Usage**
```bash
# Reduce processing
ros2 launch realsense2_camera rs_launch.py \
    spatial_filter.enable:=false \
    temporal_filter.enable:=false \
    hole_filling_filter.enable:=false
```

### **USB Bandwidth Issues**
```bash
# Check USB tree
lsusb -t

# Move to different USB port
# Use USB 3.0 port if available
# Avoid USB hubs when possible
```

## 🎯 **Performance Targets**

| Setting | Resolution | FPS | CPU Usage | Reliability |
|---------|------------|-----|-----------|-------------|
| **Minimal** | 424x240 | 10 | ~15% | Excellent |
| **Balanced** | 424x240 | 15 | ~25% | Very Good |
| **Standard** | 640x480 | 15 | ~35% | Good |
| **High** | 640x480 | 30 | ~50% | Variable |

## 📝 **Quick Commands**

```bash
# Restart camera if frozen
sudo pkill -f realsense2_camera_node
ros2 launch orbibot_navigation realsense_reliable.launch.py

# Check topics quickly
ros2 topic list | grep -E "(camera|depth|color)" 

# Test depth data
ros2 run image_view image_view image:=/camera/depth/image_rect_raw

# Test color data  
ros2 run image_view image_view image:=/camera/color/image_raw
```

Choose the **realsense_reliable.launch.py** for stable operation on your Raspberry Pi 5!