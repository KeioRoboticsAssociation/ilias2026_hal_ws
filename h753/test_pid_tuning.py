#!/usr/bin/env python3
"""
PID Tuning Test Script for H753 STM32 Motor Control System

This script demonstrates how to read and write PID parameters via MAVLink
for RoboMaster and DC motors.

Usage:
    python3 test_pid_tuning.py
"""

import time
import sys
from pymavlink import mavutil

# Connection settings
BOARD_IP = '192.168.11.4'
MAVLINK_PORT = 14550

class PIDTuner:
    """MAVLink-based PID parameter tuner"""

    def __init__(self, connection_string):
        """
        Initialize MAVLink connection

        Args:
            connection_string: MAVLink connection string (e.g., 'udp:192.168.11.4:14550')
        """
        print(f"Connecting to {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"Connected to system {self.master.target_system} component {self.master.target_component}")

        self.params = {}

    def request_all_parameters(self):
        """Request list of all parameters"""
        print("\nRequesting parameter list...")
        self.master.mav.param_request_list_send(
            self.master.target_system,
            self.master.target_component
        )

        # Receive all parameters
        param_count = 0
        timeout = time.time() + 10  # 10 second timeout

        while time.time() < timeout:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg:
                param_id = msg.param_id
                param_value = msg.param_value
                param_index = msg.param_index
                param_total = msg.param_count

                self.params[param_id] = param_value
                param_count += 1

                print(f"  [{param_index + 1}/{param_total}] {param_id} = {param_value}")

                if param_count >= param_total:
                    break

        print(f"\nReceived {param_count} parameters")
        return self.params

    def get_parameter(self, param_name):
        """
        Get a specific parameter value

        Args:
            param_name: Parameter name (e.g., "RM_20_SPD_KP")

        Returns:
            Parameter value or None if not found
        """
        print(f"\nRequesting parameter: {param_name}")
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode('utf-8'),
            -1
        )

        msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id == param_name:
            print(f"  {param_name} = {msg.param_value}")
            return msg.param_value
        else:
            print(f"  Parameter {param_name} not found")
            return None

    def set_parameter(self, param_name, value):
        """
        Set a parameter value

        Args:
            param_name: Parameter name (e.g., "RM_20_SPD_KP")
            value: New parameter value (float)

        Returns:
            True if successful, False otherwise
        """
        print(f"\nSetting parameter: {param_name} = {value}")
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode('utf-8'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        # Wait for confirmation
        msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id == param_name:
            actual_value = msg.param_value
            print(f"  Confirmed: {param_name} = {actual_value}")

            if abs(actual_value - value) < 0.001:
                print("  ✓ Value set successfully")
                return True
            else:
                print(f"  ⚠ Value was clamped: requested {value}, got {actual_value}")
                return True
        else:
            print("  ✗ Failed to set parameter")
            return False

    def tune_robomaster_pid(self, motor_id, speed_kp=None, speed_ki=None, speed_kd=None,
                           angle_kp=None, angle_ki=None, angle_kd=None):
        """
        Tune RoboMaster motor PID gains

        Args:
            motor_id: Motor ID (e.g., 20, 21)
            speed_kp, speed_ki, speed_kd: Speed PID gains (optional)
            angle_kp, angle_ki, angle_kd: Angle PID gains (optional)
        """
        print(f"\n{'='*60}")
        print(f"Tuning RoboMaster Motor {motor_id}")
        print(f"{'='*60}")

        # Speed PID
        if speed_kp is not None:
            self.set_parameter(f"RM_{motor_id}_SPD_KP", speed_kp)
        if speed_ki is not None:
            self.set_parameter(f"RM_{motor_id}_SPD_KI", speed_ki)
        if speed_kd is not None:
            self.set_parameter(f"RM_{motor_id}_SPD_KD", speed_kd)

        # Angle PID
        if angle_kp is not None:
            self.set_parameter(f"RM_{motor_id}_ANG_KP", angle_kp)
        if angle_ki is not None:
            self.set_parameter(f"RM_{motor_id}_ANG_KI", angle_ki)
        if angle_kd is not None:
            self.set_parameter(f"RM_{motor_id}_ANG_KD", angle_kd)

    def tune_dc_motor_pid(self, motor_id, speed_kp=None, speed_ki=None, speed_kd=None,
                         pos_kp=None, pos_ki=None, pos_kd=None):
        """
        Tune DC motor PID gains

        Args:
            motor_id: Motor ID (e.g., 10, 11)
            speed_kp, speed_ki, speed_kd: Speed PID gains (optional)
            pos_kp, pos_ki, pos_kd: Position PID gains (optional)
        """
        print(f"\n{'='*60}")
        print(f"Tuning DC Motor {motor_id}")
        print(f"{'='*60}")

        # Speed PID
        if speed_kp is not None:
            self.set_parameter(f"DC_{motor_id}_SPD_KP", speed_kp)
        if speed_ki is not None:
            self.set_parameter(f"DC_{motor_id}_SPD_KI", speed_ki)
        if speed_kd is not None:
            self.set_parameter(f"DC_{motor_id}_SPD_KD", speed_kd)

        # Position PID
        if pos_kp is not None:
            self.set_parameter(f"DC_{motor_id}_POS_KP", pos_kp)
        if pos_ki is not None:
            self.set_parameter(f"DC_{motor_id}_POS_KI", pos_ki)
        if pos_kd is not None:
            self.set_parameter(f"DC_{motor_id}_POS_KD", pos_kd)

    def display_pid_parameters(self):
        """Display all PID parameters in a formatted table"""
        print(f"\n{'='*60}")
        print("Current PID Parameters")
        print(f"{'='*60}")

        # Group parameters by motor
        motors = {}
        for param_name, value in sorted(self.params.items()):
            # Parse parameter name (e.g., "RM_20_SPD_KP")
            parts = param_name.split('_')
            if len(parts) >= 3:
                motor_type = parts[0]  # RM or DC
                motor_id = parts[1]    # 20, 21, 10, etc.
                param_type = '_'.join(parts[2:])  # SPD_KP, ANG_KI, etc.

                key = f"{motor_type}_{motor_id}"
                if key not in motors:
                    motors[key] = {}
                motors[key][param_type] = value

        # Display grouped parameters
        for motor_key in sorted(motors.keys()):
            print(f"\n{motor_key}:")
            params = motors[motor_key]
            for param_type in sorted(params.keys()):
                print(f"  {param_type:12} = {params[param_type]:8.4f}")

    def close(self):
        """Close MAVLink connection"""
        self.master.close()


def interactive_tuning_session():
    """Interactive PID tuning session"""
    connection_string = f'udp:{BOARD_IP}:{MAVLINK_PORT}'
    tuner = PIDTuner(connection_string)

    # Request all parameters
    tuner.request_all_parameters()
    tuner.display_pid_parameters()

    print(f"\n{'='*60}")
    print("Interactive PID Tuning Session")
    print(f"{'='*60}")
    print("\nCommands:")
    print("  list               - List all parameters")
    print("  get <param>        - Get parameter value")
    print("  set <param> <val>  - Set parameter value")
    print("  rm <id> <gains>    - Tune RoboMaster motor (e.g., 'rm 20 speed_kp=55.0')")
    print("  dc <id> <gains>    - Tune DC motor (e.g., 'dc 10 speed_kp=0.15')")
    print("  save               - Save parameters to Flash (TODO)")
    print("  quit               - Exit")
    print()

    while True:
        try:
            cmd = input(">>> ").strip().split()
            if not cmd:
                continue

            if cmd[0] == 'quit' or cmd[0] == 'q':
                break

            elif cmd[0] == 'list' or cmd[0] == 'ls':
                tuner.request_all_parameters()
                tuner.display_pid_parameters()

            elif cmd[0] == 'get' and len(cmd) >= 2:
                param_name = cmd[1]
                tuner.get_parameter(param_name)

            elif cmd[0] == 'set' and len(cmd) >= 3:
                param_name = cmd[1]
                value = float(cmd[2])
                tuner.set_parameter(param_name, value)

            elif cmd[0] == 'rm' and len(cmd) >= 3:
                motor_id = int(cmd[1])
                gains = {}
                for arg in cmd[2:]:
                    key, val = arg.split('=')
                    gains[key] = float(val)
                tuner.tune_robomaster_pid(motor_id, **gains)

            elif cmd[0] == 'dc' and len(cmd) >= 3:
                motor_id = int(cmd[1])
                gains = {}
                for arg in cmd[2:]:
                    key, val = arg.split('=')
                    gains[key] = float(val)
                tuner.tune_dc_motor_pid(motor_id, **gains)

            elif cmd[0] == 'save':
                print("⚠ Flash save not yet implemented in firmware")

            else:
                print("Unknown command. Type 'quit' to exit.")

        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except Exception as e:
            print(f"Error: {e}")

    tuner.close()
    print("\nConnection closed")


def automated_tuning_demo():
    """Automated PID tuning demonstration"""
    connection_string = f'udp:{BOARD_IP}:{MAVLINK_PORT}'
    tuner = PIDTuner(connection_string)

    print(f"\n{'='*60}")
    print("Automated PID Tuning Demo")
    print(f"{'='*60}")

    # Get current parameters
    print("\n1. Reading current parameters...")
    tuner.request_all_parameters()
    tuner.display_pid_parameters()

    # Example: Tune RoboMaster motor 20 (GM6020)
    print("\n2. Tuning RoboMaster Motor 20 (GM6020)...")
    tuner.tune_robomaster_pid(
        motor_id=20,
        speed_kp=55.0,  # Increase speed Kp
        speed_ki=0.15,  # Add some integral
        angle_kp=0.12   # Slightly increase angle Kp
    )

    # Example: Tune DC motor 10
    print("\n3. Tuning DC Motor 10...")
    tuner.tune_dc_motor_pid(
        motor_id=10,
        speed_kp=0.12,
        speed_ki=0.06,
        pos_kp=0.55
    )

    # Verify changes
    print("\n4. Verifying changes...")
    time.sleep(0.5)
    tuner.request_all_parameters()
    tuner.display_pid_parameters()

    tuner.close()
    print("\nDemo complete!")


def main():
    """Main entry point"""
    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        automated_tuning_demo()
    else:
        interactive_tuning_session()


if __name__ == '__main__':
    main()
