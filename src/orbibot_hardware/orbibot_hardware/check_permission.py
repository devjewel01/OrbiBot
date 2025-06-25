#!/usr/bin/env python3
"""
Check Permissions Utility
Checks device permissions and hardware accessibility
"""

import os
import sys
import subprocess
import rclpy
from rclpy.node import Node


class PermissionsChecker(Node):
    """Check device permissions and hardware access"""
    
    def __init__(self):
        super().__init__('permissions_checker')
        
    def check_permissions(self):
        """Check all device permissions and accessibility"""
        self.get_logger().info("=== OrbiBot Permissions Check ===")
        
        all_good = True
        
        # Check device files
        devices = {
            '/dev/motordriver': 'Motor driver (CH340)',
            '/dev/lidar': 'LIDAR (CP210x)'
        }
        
        for device, description in devices.items():
            if os.path.exists(device):
                # Check if readable/writable
                if os.access(device, os.R_OK | os.W_OK):
                    self.get_logger().info(f"✓ {description}: {device} - OK")
                else:
                    self.get_logger().error(f"✗ {description}: {device} - No permission")
                    all_good = False
            else:
                self.get_logger().warn(f"⚠ {description}: {device} - Not found")
                all_good = False
        
        # Check USB devices
        self.check_usb_devices()
        
        # Check ROSMaster_Lib
        self.check_rosmaster_lib()
        
        # Check user groups
        self.check_user_groups()
        
        if all_good:
            self.get_logger().info("✓ All permissions look good!")
        else:
            self.get_logger().warn("⚠ Some issues found - check setup_permissions.sh")
            
        return all_good
    
    def check_usb_devices(self):
        """Check for connected USB devices"""
        try:
            result = subprocess.run(['lsusb'], capture_output=True, text=True)
            usb_output = result.stdout
            
            # Look for known devices
            devices_found = []
            if '1a86' in usb_output:  # CH340
                devices_found.append("CH340 (Motor driver)")
            if '10c4' in usb_output:  # CP210x
                devices_found.append("CP210x (LIDAR)")
            if '8086' in usb_output:  # Intel RealSense
                devices_found.append("Intel RealSense")
            
            if devices_found:
                self.get_logger().info(f"✓ USB devices found: {', '.join(devices_found)}")
            else:
                self.get_logger().warn("⚠ No expected USB devices found")
                
        except Exception as e:
            self.get_logger().error(f"Could not check USB devices: {str(e)}")
    
    def check_rosmaster_lib(self):
        """Check ROSMaster_Lib availability"""
        try:
            import Rosmaster_Lib
            self.get_logger().info("✓ ROSMaster_Lib is available")
            return True
        except ImportError:
            self.get_logger().error("✗ ROSMaster_Lib not found - install Yahboom drivers")
            return False
    
    def check_user_groups(self):
        """Check if user is in required groups"""
        try:
            result = subprocess.run(['groups'], capture_output=True, text=True)
            groups = result.stdout.strip()
            
            required_groups = ['dialout', 'plugdev']
            missing_groups = []
            
            for group in required_groups:
                if group not in groups:
                    missing_groups.append(group)
            
            if missing_groups:
                self.get_logger().warn(f"⚠ User not in groups: {', '.join(missing_groups)}")
                self.get_logger().info("Run: sudo usermod -a -G dialout,plugdev $USER")
                self.get_logger().info("Then log out and back in")
            else:
                self.get_logger().info("✓ User is in required groups")
                
        except Exception as e:
            self.get_logger().error(f"Could not check user groups: {str(e)}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        checker = PermissionsChecker()
        success = checker.check_permissions()
        
        if not success:
            sys.exit(1)
            
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()