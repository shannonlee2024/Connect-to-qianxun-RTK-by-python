Here is the complete Markdown content, ready to be copied into a file named **`USB_SETUP.md`** or added to your **`README.md`**.

```markdown
# 🛠️ USB Device Persistent Configuration Guide

This guide explains how to configure **Persistent USB Symlinks** for your ROS robotic setup. 

## ⚠️ The Problem
On Linux/ROS systems, USB devices are assigned names like `/dev/ttyACM0` or `/dev/ttyACM1` based on the order they are plugged in or detected. If the system reboots or devices are replugged, these names can swap, causing ROS nodes (MAVROS, Ublox Driver) to fail because they are looking for the wrong port.

## ✅ The Solution
We use `udev` rules to assign a **permanent custom name** (Symlink) to each device based on its unique Hardware ID.
- **GPS** will always be mapped to: `/dev/ublox`
- **Flight Controller** will always be mapped to: `/dev/pixhawk`

---

## 1. Ublox GPS Setup

### Step 1: Identify the Device ID
1. Connect **only** the Ublox GPS via USB (unplug the Flight Controller to avoid confusion).
2. Check the current device name:
   ```bash
   ls /dev/ttyACM*
   ```
3. Get the **Vendor ID** and **Product ID** (assuming the device is `ttyACM0`):
   ```bash
   udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct" | head -n 2
   ```
   *Example Output (yours may differ):*
   > ATTRS{idVendor}=="1546"  
   > ATTRS{idProduct}=="01a8"

### Step 2: Create the Rule
1. Create a new rule file:
   ```bash
   sudo nano /etc/udev/rules.d/99-ublox.rules
   ```
2. Paste the following line (**Replace `idVendor` and `idProduct` with YOUR values found above**):
   ```bash
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", MODE="0666", SYMLINK+="ublox"
   ```
3. Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

---

## 2. PX4 Flight Controller Setup

### Step 1: Identify the Device ID
1. Connect **only** the PX4/Pixhawk Flight Controller via USB.
2. Get the **Vendor ID** and **Product ID** (assuming the device is `ttyACM0`):
   ```bash
   udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct" | head -n 2
   ```
   *Example Output (yours may differ):*
   > ATTRS{idVendor}=="26ac"  
   > ATTRS{idProduct}=="0011"

### Step 2: Create the Rule
1. Create a new rule file:
   ```bash
   sudo nano /etc/udev/rules.d/99-pixhawk.rules
   ```
2. Paste the following line (**Replace `idVendor` and `idProduct` with YOUR values**):
   ```bash
   SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", MODE="0666", SYMLINK+="pixhawk"
   ```
3. Save and exit.

---

## 3. Apply Changes & Verify

1. **Reload rules and trigger:**
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
2. **Unplug both devices and plug them back in.**
3. **Verify the shortcuts exist:**
   ```bash
   ls -l /dev/ublox
   ls -l /dev/pixhawk
   ```
   You should see them pointing to `ttyACM...` (e.g., `lrwxrwxrwx ... /dev/ublox -> ttyACM0`).

---

## 4. Update ROS Launch Files

Now that the names are fixed, update your launch files to use these permanent names.

### 🔹 For Ublox Driver (`ublox_driver.launch`)
Modify the `device` argument to use the new symlink:
```xml
<arg name="device" default="/dev/ublox" />
```

### 🔹 For MAVROS (`px4.launch`)
Modify the `fcu_url` argument:
```xml
<!-- Baud rate 921600 is recommended for direct USB connection -->
<arg name="fcu_url" default="/dev/pixhawk:921600" />
```

---
**🎉 Done! Your hardware configuration is now robust and reboot-proof.**
```
