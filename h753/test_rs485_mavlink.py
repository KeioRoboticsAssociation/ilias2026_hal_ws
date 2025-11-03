#!/usr/bin/env python3
"""
RS485 Motor Control Test via MAVLink

This script demonstrates how to control RS485 motors (Ikeya MD) connected to
STM32H753 via MAVLink MOTOR_COMMAND messages over UDP.

Requirements:
    pip install pymavlink

Motor Configuration:
    - Motor ID 30: RS485 device 1 on USART1 (Velocity mode, RPS)
    - Motor ID 31: RS485 device 2 on USART2 (Position mode)

Usage:
    python3 test_rs485_mavlink.py
"""

from pymavlink import mavutil
import time
import sys
import argparse


class RS485MotorController:
    """
    Controller for RS485 motors via MAVLink MOTOR_COMMAND messages
    """

    # Control modes (from motor_config.h)
    CONTROL_MODE_POSITION = 0
    CONTROL_MODE_VELOCITY = 1
    CONTROL_MODE_CURRENT = 2
    CONTROL_MODE_DUTY_CYCLE = 3

    def __init__(self, connection_string='udp:192.168.11.4:14550'):
        """
        Initialize MAVLink connection to STM32H753

        Args:
            connection_string: MAVLink connection string (UDP or serial)
        """
        print(f"Connecting to {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"Connected! System ID: {self.master.target_system}, "
              f"Component ID: {self.master.target_component}")

    def send_motor_command(self, motor_id, control_mode, target_value, enable=True):
        """
        Send MOTOR_COMMAND message to control a motor

        Args:
            motor_id: Motor ID (30 for RS485 #1, 31 for RS485 #2)
            control_mode: Control mode (POSITION, VELOCITY, CURRENT, DUTY_CYCLE)
            target_value: Target value (units depend on control mode)
            enable: Enable (True) or disable (False) motor
        """
        self.master.mav.motor_command_send(
            motor_id=motor_id,
            control_mode=control_mode,
            target_value=float(target_value),
            enable=1 if enable else 0
        )
        print(f"Sent: Motor {motor_id}, Mode {control_mode}, "
              f"Target {target_value:.2f}, Enable {enable}")

    def set_velocity(self, motor_id, rps):
        """
        Set motor velocity in RPS (Revolutions Per Second)

        Args:
            motor_id: Motor ID (typically 30)
            rps: Target velocity in RPS (-100.0 to +100.0)
        """
        self.send_motor_command(motor_id, self.CONTROL_MODE_VELOCITY, rps, True)

    def set_position(self, motor_id, position_rad):
        """
        Set motor position in radians

        Args:
            motor_id: Motor ID (typically 31)
            position_rad: Target position in radians
        """
        self.send_motor_command(motor_id, self.CONTROL_MODE_POSITION, position_rad, True)

    def stop_motor(self, motor_id):
        """
        Stop a motor by setting velocity to 0 or disabling it

        Args:
            motor_id: Motor ID to stop
        """
        self.send_motor_command(motor_id, self.CONTROL_MODE_VELOCITY, 0.0, True)
        print(f"Motor {motor_id} stopped")

    def emergency_stop_all(self):
        """
        Emergency stop for all motors
        """
        print("EMERGENCY STOP - Disabling all motors")
        for motor_id in [30, 31]:
            self.send_motor_command(motor_id, self.CONTROL_MODE_VELOCITY, 0.0, False)

    def close(self):
        """
        Close MAVLink connection
        """
        self.master.close()
        print("Connection closed")


def test_velocity_control(controller):
    """
    Test RS485 motor velocity control (Motor ID 30)
    """
    print("\n" + "="*60)
    print("TEST 1: Velocity Control (Motor 30 - RS485 Device 1)")
    print("="*60)

    motor_id = 30

    # Forward rotation at 20 RPS
    print("\n→ Step 1: Forward rotation at 20 RPS")
    controller.set_velocity(motor_id, 20.0)
    time.sleep(3)

    # Increase to 50 RPS
    print("\n→ Step 2: Increase to 50 RPS")
    controller.set_velocity(motor_id, 50.0)
    time.sleep(3)

    # Reverse rotation at -30 RPS
    print("\n→ Step 3: Reverse rotation at -30 RPS")
    controller.set_velocity(motor_id, -30.0)
    time.sleep(3)

    # Stop motor
    print("\n→ Step 4: Stop motor")
    controller.stop_motor(motor_id)
    time.sleep(1)

    print("\n✓ Velocity control test completed")


def test_position_control(controller):
    """
    Test RS485 motor position control (Motor ID 31)
    """
    print("\n" + "="*60)
    print("TEST 2: Position Control (Motor 31 - RS485 Device 2)")
    print("="*60)

    motor_id = 31

    # Move to 0 radians (home position)
    print("\n→ Step 1: Move to home position (0 rad)")
    controller.set_position(motor_id, 0.0)
    time.sleep(3)

    # Move to π/2 radians (90 degrees)
    print("\n→ Step 2: Move to π/2 rad (90°)")
    controller.set_position(motor_id, 3.14159 / 2)
    time.sleep(3)

    # Move to π radians (180 degrees)
    print("\n→ Step 3: Move to π rad (180°)")
    controller.set_position(motor_id, 3.14159)
    time.sleep(3)

    # Move to -π/2 radians (-90 degrees)
    print("\n→ Step 4: Move to -π/2 rad (-90°)")
    controller.set_position(motor_id, -3.14159 / 2)
    time.sleep(3)

    # Return to home
    print("\n→ Step 5: Return to home position (0 rad)")
    controller.set_position(motor_id, 0.0)
    time.sleep(2)

    print("\n✓ Position control test completed")


def test_simultaneous_control(controller):
    """
    Test controlling both motors simultaneously
    """
    print("\n" + "="*60)
    print("TEST 3: Simultaneous Control (Both Motors)")
    print("="*60)

    print("\n→ Motor 30: Velocity 25 RPS")
    controller.set_velocity(30, 25.0)

    print("→ Motor 31: Position π/4 rad (45°)")
    controller.set_position(31, 3.14159 / 4)

    time.sleep(4)

    print("\n→ Reversing motor 30 to -25 RPS")
    controller.set_velocity(30, -25.0)

    print("→ Moving motor 31 to -π/4 rad (-45°)")
    controller.set_position(31, -3.14159 / 4)

    time.sleep(4)

    print("\n→ Stopping both motors")
    controller.stop_motor(30)
    controller.stop_motor(31)

    print("\n✓ Simultaneous control test completed")


def interactive_mode(controller):
    """
    Interactive control mode
    """
    print("\n" + "="*60)
    print("INTERACTIVE MODE")
    print("="*60)
    print("\nCommands:")
    print("  v <motor_id> <rps>      - Set velocity (e.g., 'v 30 50.0')")
    print("  p <motor_id> <rad>      - Set position (e.g., 'p 31 1.57')")
    print("  s <motor_id>            - Stop motor (e.g., 's 30')")
    print("  e                       - Emergency stop all")
    print("  q                       - Quit")
    print()

    while True:
        try:
            cmd = input(">>> ").strip().split()
            if not cmd:
                continue

            if cmd[0] == 'q':
                break
            elif cmd[0] == 'e':
                controller.emergency_stop_all()
            elif cmd[0] == 'v' and len(cmd) == 3:
                motor_id = int(cmd[1])
                rps = float(cmd[2])
                controller.set_velocity(motor_id, rps)
            elif cmd[0] == 'p' and len(cmd) == 3:
                motor_id = int(cmd[1])
                rad = float(cmd[2])
                controller.set_position(motor_id, rad)
            elif cmd[0] == 's' and len(cmd) == 2:
                motor_id = int(cmd[1])
                controller.stop_motor(motor_id)
            else:
                print("Invalid command")
        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    """
    Main test program
    """
    parser = argparse.ArgumentParser(description='RS485 Motor Control Test via MAVLink')
    parser.add_argument('--ip', default='192.168.11.4',
                        help='STM32 IP address (default: 192.168.11.4)')
    parser.add_argument('--port', type=int, default=14550,
                        help='MAVLink port (default: 14550)')
    parser.add_argument('--mode', choices=['test', 'interactive'], default='test',
                        help='Test mode: automated tests or interactive control')

    args = parser.parse_args()

    connection_string = f'udp:{args.ip}:{args.port}'

    try:
        # Create controller
        controller = RS485MotorController(connection_string)

        if args.mode == 'test':
            # Run automated tests
            test_velocity_control(controller)
            time.sleep(2)

            test_position_control(controller)
            time.sleep(2)

            test_simultaneous_control(controller)

            print("\n" + "="*60)
            print("ALL TESTS COMPLETED SUCCESSFULLY")
            print("="*60)

        else:
            # Interactive mode
            interactive_mode(controller)

        # Final cleanup
        print("\nStopping all motors...")
        controller.emergency_stop_all()
        time.sleep(1)

        controller.close()

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        if 'controller' in locals():
            controller.emergency_stop_all()
            controller.close()
    except Exception as e:
        print(f"\nError: {e}")
        if 'controller' in locals():
            controller.emergency_stop_all()
            controller.close()
        sys.exit(1)


if __name__ == '__main__':
    main()
