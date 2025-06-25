#!/usr/bin/env python3
"""
API Debug Utility
Checks the exact method signatures of ROSMaster_Lib
"""

import inspect


def debug_api_signatures():
    """Debug ROSMaster_Lib method signatures"""
    try:
        import Rosmaster_Lib
        
        print("=== ROSMaster_Lib API Signature Debug ===")
        
        # Connect to board
        board = Rosmaster_Lib.Rosmaster(com="/dev/motordriver")
        print("✓ Connected to ROSMaster board")
        
        # Check set_motor signature
        try:
            sig = inspect.signature(board.set_motor)
            print(f"set_motor signature: {sig}")
        except Exception as e:
            print(f"Could not get set_motor signature: {e}")
        
        # Check other motor-related methods
        motor_methods = ['set_motor', 'set_car_motion', 'set_car_run']
        
        for method_name in motor_methods:
            if hasattr(board, method_name):
                try:
                    method = getattr(board, method_name)
                    sig = inspect.signature(method)
                    print(f"{method_name} signature: {sig}")
                except Exception as e:
                    print(f"Could not get {method_name} signature: {e}")
            else:
                print(f"{method_name}: NOT FOUND")
        
        # Test different set_motor calls
        print("\n=== Testing set_motor calls ===")
        
        # Test 1: 4 parameters
        try:
            print("Testing: set_motor(0, 0, 0, 0)")
            board.set_motor(0, 0, 0, 0)
            print("✓ set_motor(0, 0, 0, 0) works")
        except Exception as e:
            print(f"✗ set_motor(0, 0, 0, 0) failed: {e}")
        
        # Test 2: Different parameter count
        try:
            print("Testing: set_motor(0)")
            board.set_motor(0)
            print("✓ set_motor(0) works")
        except Exception as e:
            print(f"✗ set_motor(0) failed: {e}")
        
        # Test 3: Try with list
        try:
            print("Testing: set_motor([0, 0, 0, 0])")
            board.set_motor([0, 0, 0, 0])
            print("✓ set_motor([0, 0, 0, 0]) works")
        except Exception as e:
            print(f"✗ set_motor([0, 0, 0, 0]) failed: {e}")
        
        # Test alternative methods
        print("\n=== Testing alternative motor methods ===")
        
        try:
            print("Testing: set_car_motion(0, 0, 0)")
            board.set_car_motion(0, 0, 0)
            print("✓ set_car_motion(0, 0, 0) works")
        except Exception as e:
            print(f"✗ set_car_motion(0, 0, 0) failed: {e}")
        
        try:
            print("Testing: set_car_run(0, 0)")
            board.set_car_run(0, 0)
            print("✓ set_car_run(0, 0) works")
        except Exception as e:
            print(f"✗ set_car_run(0, 0) failed: {e}")
        
        print("\n=== Other API Tests ===")
        
        # Test encoder method
        try:
            encoders = board.get_motor_encoder()
            print(f"✓ get_motor_encoder() returns: {encoders} (type: {type(encoders)})")
        except Exception as e:
            print(f"✗ get_motor_encoder() failed: {e}")
        
        # Test IMU methods
        try:
            accel = board.get_accelerometer_data()
            print(f"✓ get_accelerometer_data() returns: {accel} (type: {type(accel)})")
        except Exception as e:
            print(f"✗ get_accelerometer_data() failed: {e}")
        
        try:
            voltage = board.get_battery_voltage()
            print(f"✓ get_battery_voltage() returns: {voltage} (type: {type(voltage)})")
        except Exception as e:
            print(f"✗ get_battery_voltage() failed: {e}")
        
    except ImportError:
        print("✗ Cannot import ROSMaster_Lib")
    except Exception as e:
        print(f"✗ Error: {e}")


def main():
    """Main function"""
    print("=== ROSMaster API Debug Utility ===")
    debug_api_signatures()


if __name__ == '__main__':
    main()