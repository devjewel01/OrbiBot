#!/usr/bin/env python3
"""
ROSMaster_Lib Debug Utility
Helps debug ROSMaster_Lib connection issues
"""

import sys
import inspect


def debug_rosmaster_lib():
    """Debug ROSMaster_Lib to understand its interface"""
    try:
        import Rosmaster_Lib
        print("✓ ROSMaster_Lib imported successfully")
        
        # Inspect the Rosmaster class
        print("\n=== ROSMaster Class Information ===")
        rosmaster_class = Rosmaster_Lib.Rosmaster
        
        # Get constructor signature
        try:
            sig = inspect.signature(rosmaster_class.__init__)
            print(f"Constructor signature: {sig}")
        except Exception as e:
            print(f"Could not get constructor signature: {e}")
        
        # Get class methods
        methods = [method for method in dir(rosmaster_class) if not method.startswith('_')]
        print(f"Available methods: {methods}")
        
        # Try to create instance with different parameters
        print("\n=== Testing Different Initialization Methods ===")
        
        # Method 1: Default constructor
        try:
            board1 = Rosmaster_Lib.Rosmaster()
            print("✓ Default constructor works")
            
            # Check attributes
            attrs = [attr for attr in dir(board1) if not attr.startswith('_')]
            print(f"Instance attributes: {attrs}")
            
            # Check for serial-related attributes
            serial_attrs = [attr for attr in attrs if 'ser' in attr.lower() or 'port' in attr.lower()]
            print(f"Serial-related attributes: {serial_attrs}")
            
            # Try to access serial port info
            if hasattr(board1, 'ser'):
                print(f"Serial object: {board1.ser}")
                if hasattr(board1.ser, 'port'):
                    print(f"Current port: {board1.ser.port}")
            
            del board1
            
        except Exception as e:
            print(f"✗ Default constructor failed: {e}")
        
        # Method 2: With com parameter
        try:
            board2 = Rosmaster_Lib.Rosmaster(com="/dev/motordriver")
            print("✓ Constructor with com='/dev/motordriver' works")
            del board2
        except Exception as e:
            print(f"✗ Constructor with com parameter failed: {e}")
        
        # Method 3: With port parameter
        try:
            board3 = Rosmaster_Lib.Rosmaster(port="/dev/motordriver")
            print("✓ Constructor with port='/dev/motordriver' works")
            del board3
        except Exception as e:
            print(f"✗ Constructor with port parameter failed: {e}")
        
        # Method 4: Check if there are other parameter names
        try:
            board4 = Rosmaster_Lib.Rosmaster(device="/dev/motordriver")
            print("✓ Constructor with device='/dev/motordriver' works")
            del board4
        except Exception as e:
            print(f"✗ Constructor with device parameter failed: {e}")
        
        print("\n=== Checking Serial Ports ===")
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"Available port: {port.device} - {port.description}")
        
        # Check if our device exists
        import os
        if os.path.exists("/dev/motordriver"):
            print("✓ /dev/motordriver exists")
            try:
                import serial
                ser = serial.Serial("/dev/motordriver", 115200, timeout=1)
                print("✓ Can open /dev/motordriver directly")
                ser.close()
            except Exception as e:
                print(f"✗ Cannot open /dev/motordriver: {e}")
        else:
            print("✗ /dev/motordriver does not exist")
            
    except ImportError as e:
        print(f"✗ Cannot import ROSMaster_Lib: {e}")
    except Exception as e:
        print(f"✗ Unexpected error: {e}")


def main():
    """Main function"""
    print("=== ROSMaster_Lib Debug Utility ===")
    debug_rosmaster_lib()


if __name__ == '__main__':
    main()