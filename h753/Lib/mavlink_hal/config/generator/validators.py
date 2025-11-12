#!/usr/bin/env python3
"""
Configuration Validators

Additional validation beyond JSON schema, including:
- Motor ID uniqueness and range checking
- Hardware resource conflict detection
- Pin assignment validation
- MAVLink compatibility checks
"""

from typing import Dict, List, Any, Set, Tuple, Optional
from dataclasses import dataclass


@dataclass
class ValidationWarning:
    """Represents a validation warning"""
    level: str  # 'warning' or 'error'
    message: str
    location: str
    suggestion: Optional[str] = None


class ConfigValidators:
    """Configuration validation utilities"""

    # Motor ID ranges (from h753 conventions)
    MOTOR_ID_RANGES = {
        'servo': (1, 9),
        'dc_motor': (10, 15),
        'robomaster': (20, 29),
        'rs485_motor': (30, 49),
        'bldc_motor': (50, 59),
        'stepper': (60, 69),
    }

    SENSOR_ID_MIN = 100

    def __init__(self, config: Dict[str, Any]):
        """Initialize validator with configuration

        Args:
            config: Parsed configuration dictionary
        """
        self.config = config
        self.warnings: List[ValidationWarning] = []
        self.errors: List[ValidationWarning] = []

    def validate_all(self) -> Tuple[List[ValidationWarning], List[ValidationWarning]]:
        """Run all validation checks

        Returns:
            Tuple of (warnings, errors)
        """
        self.warnings = []
        self.errors = []

        # Run all validation checks
        self._validate_motor_ids()
        self._validate_hardware_resources()
        self._validate_pin_assignments()
        self._validate_mavlink_config()
        self._validate_safety_limits()
        self._validate_platform_compatibility()

        return self.warnings, self.errors

    def _add_warning(self, message: str, location: str, suggestion: str = None):
        """Add a validation warning"""
        self.warnings.append(ValidationWarning(
            level='warning',
            message=message,
            location=location,
            suggestion=suggestion
        ))

    def _add_error(self, message: str, location: str, suggestion: str = None):
        """Add a validation error"""
        self.errors.append(ValidationWarning(
            level='error',
            message=message,
            location=location,
            suggestion=suggestion
        ))

    def _validate_motor_ids(self):
        """Validate motor IDs for uniqueness and range"""
        devices = self.config.get('devices', [])

        seen_ids: Set[int] = set()
        motor_types = ['servo', 'dc_motor', 'bldc_motor', 'stepper',
                       'robomaster', 'rs485_motor']

        for i, device in enumerate(devices):
            device_id = device.get('id')
            device_type = device.get('type')
            device_name = device.get('name', f'device_{i}')
            location = f"devices[{i}] ({device_name})"

            # Check ID uniqueness
            if device_id in seen_ids:
                self._add_error(
                    f"Duplicate device ID: {device_id}",
                    location,
                    "Each device must have a unique ID"
                )
            seen_ids.add(device_id)

            # Check ID ranges for motors
            if device_type in motor_types:
                expected_range = self.MOTOR_ID_RANGES.get(device_type)
                if expected_range:
                    min_id, max_id = expected_range
                    if not (min_id <= device_id <= max_id):
                        self._add_warning(
                            f"{device_type} ID {device_id} is outside conventional "
                            f"range ({min_id}-{max_id})",
                            location,
                            f"Consider using IDs in range {min_id}-{max_id} for {device_type}"
                        )

            # Check sensor IDs
            if device_type in ['encoder', 'imu', 'gps', 'analog_sensor', 'digital_io']:
                if device_id < self.SENSOR_ID_MIN:
                    self._add_warning(
                        f"Sensor ID {device_id} is below conventional minimum ({self.SENSOR_ID_MIN})",
                        location,
                        f"Consider using IDs >= {self.SENSOR_ID_MIN} for sensors"
                    )

    def _validate_hardware_resources(self):
        """Validate hardware resource usage (timers, CAN, UART)"""
        devices = self.config.get('devices', [])
        platform_type = self.config.get('platform', {}).get('type')

        # Track resource usage
        timers_used: Dict[Tuple[int, int], List[str]] = {}  # (timer_id, channel) -> [device_names]
        can_ids_used: Dict[int, List[str]] = {}  # can_id -> [device_names]
        uart_ids_used: Dict[int, List[str]] = {}  # uart_id -> [device_names]
        pins_used: Dict[str, List[str]] = {}  # pin_name -> [device_names]

        for i, device in enumerate(devices):
            device_name = device.get('name', f'device_{i}')
            device_type = device.get('type')
            hardware = device.get('hardware', {})

            # Check timer usage (for PWM devices)
            if device_type in ['servo', 'dc_motor', 'bldc_motor']:
                timer_id = hardware.get('timer')
                channel = hardware.get('channel')

                if timer_id and channel:
                    key = (timer_id, channel)
                    if key not in timers_used:
                        timers_used[key] = []
                    timers_used[key].append(device_name)

            # Check CAN ID usage (for RoboMaster)
            if device_type == 'robomaster':
                can_id = hardware.get('can_id')
                if can_id:
                    if can_id not in can_ids_used:
                        can_ids_used[can_id] = []
                    can_ids_used[can_id].append(device_name)

            # Check UART usage (for RS485)
            if device_type == 'rs485_motor':
                uart_port = hardware.get('uart_port')
                if uart_port:
                    if uart_port not in uart_ids_used:
                        uart_ids_used[uart_port] = []
                    uart_ids_used[uart_port].append(device_name)

            # Check pin usage
            pins = hardware.get('pins', {})
            for pin_role, pin_name in pins.items():
                if pin_name:
                    if pin_name not in pins_used:
                        pins_used[pin_name] = []
                    pins_used[pin_name].append(f"{device_name}.{pin_role}")

        # Report conflicts
        for (timer_id, channel), device_list in timers_used.items():
            if len(device_list) > 1:
                self._add_error(
                    f"Timer {timer_id} channel {channel} assigned to multiple devices: "
                    f"{', '.join(device_list)}",
                    "hardware resources",
                    "Each timer channel can only be used by one device"
                )

        for can_id, device_list in can_ids_used.items():
            if len(device_list) > 1:
                self._add_error(
                    f"CAN ID {hex(can_id)} assigned to multiple devices: "
                    f"{', '.join(device_list)}",
                    "hardware resources",
                    "Each CAN ID must be unique"
                )

        for pin_name, usage_list in pins_used.items():
            if len(usage_list) > 1:
                # Some pins can be shared (e.g., I2C), but warn anyway
                self._add_warning(
                    f"Pin {pin_name} used by multiple devices: {', '.join(usage_list)}",
                    "hardware resources",
                    "Verify pin sharing is intentional (e.g., I2C bus)"
                )

    def _validate_pin_assignments(self):
        """Validate pin assignments for platform"""
        devices = self.config.get('devices', [])
        platform_type = self.config.get('platform', {}).get('type')

        # Platform-specific pin patterns
        valid_pin_patterns = {
            'stm32': r'^P[A-K]\d{1,2}$',  # PA0-PA15, PB0-PB15, etc.
            'arduino': r'^\d{1,2}$|^A\d{1}$',  # 0-53, A0-A15
            'esp32': r'^(GPIO)?\d{1,2}$',  # GPIO0-GPIO39 or 0-39
        }

        if platform_type not in valid_pin_patterns:
            return

        import re
        pattern = re.compile(valid_pin_patterns[platform_type])

        for i, device in enumerate(devices):
            device_name = device.get('name', f'device_{i}')
            hardware = device.get('hardware', {})
            pins = hardware.get('pins', {})

            for pin_role, pin_name in pins.items():
                if not pin_name:
                    continue

                if not pattern.match(pin_name):
                    self._add_warning(
                        f"Pin '{pin_name}' for {device_name}.{pin_role} doesn't match "
                        f"expected pattern for {platform_type}",
                        f"devices[{i}].hardware.pins.{pin_role}",
                        f"Expected format: {valid_pin_patterns[platform_type]}"
                    )

    def _validate_mavlink_config(self):
        """Validate MAVLink configuration"""
        mavlink = self.config.get('mavlink', {})

        if not mavlink:
            self._add_warning(
                "No MAVLink configuration found",
                "mavlink",
                "Add MAVLink transport configuration"
            )
            return

        transport = mavlink.get('transport', {})
        transport_type = transport.get('type')

        if not transport_type:
            self._add_error(
                "MAVLink transport type not specified",
                "mavlink.transport",
                "Specify 'uart', 'udp', 'tcp', or 'can'"
            )
            return

        # Validate transport-specific settings
        if transport_type == 'uart':
            uart_config = transport.get('uart', {})
            if not uart_config.get('port'):
                self._add_error(
                    "UART port not specified",
                    "mavlink.transport.uart",
                    "Specify UART port (e.g., 'USART2', '/dev/ttyUSB0')"
                )

            baudrate = uart_config.get('baudrate', 57600)
            if baudrate < 9600 or baudrate > 921600:
                self._add_warning(
                    f"Unusual UART baudrate: {baudrate}",
                    "mavlink.transport.uart.baudrate",
                    "Common MAVLink baudrates: 57600, 115200"
                )

        elif transport_type == 'udp':
            udp_config = transport.get('udp', {})
            if not udp_config.get('local_ip'):
                self._add_error(
                    "UDP local IP not specified",
                    "mavlink.transport.udp",
                    "Specify local IP address"
                )

            local_port = udp_config.get('local_port', 14550)
            if local_port != 14550:
                self._add_warning(
                    f"Non-standard MAVLink port: {local_port}",
                    "mavlink.transport.udp.local_port",
                    "Standard MAVLink UDP port is 14550"
                )

    def _validate_safety_limits(self):
        """Validate safety limits and failsafe configurations"""
        devices = self.config.get('devices', [])

        for i, device in enumerate(devices):
            device_name = device.get('name', f'device_{i}')
            device_type = device.get('type')
            limits = device.get('limits', {})
            failsafe = device.get('failsafe', {})

            # Check if safety limits are defined for motors
            if device_type in ['servo', 'dc_motor', 'bldc_motor', 'stepper', 'robomaster']:
                if not limits:
                    self._add_warning(
                        f"No safety limits defined for motor {device_name}",
                        f"devices[{i}].limits",
                        "Add limits (max_speed, max_current, max_temperature)"
                    )

                # Check if failsafe is defined
                if not failsafe:
                    self._add_warning(
                        f"No failsafe behavior defined for motor {device_name}",
                        f"devices[{i}].failsafe",
                        "Add failsafe action (hold, neutral, disable)"
                    )
                else:
                    action = failsafe.get('action')
                    if action not in ['hold', 'neutral', 'disable', 'custom']:
                        self._add_error(
                            f"Invalid failsafe action '{action}' for {device_name}",
                            f"devices[{i}].failsafe.action",
                            "Use: hold, neutral, disable, or custom"
                        )

    def _validate_platform_compatibility(self):
        """Validate device compatibility with platform"""
        platform_type = self.config.get('platform', {}).get('type')
        devices = self.config.get('devices', [])

        # Platform capabilities
        platform_capabilities = {
            'stm32': {'servo', 'dc_motor', 'bldc_motor', 'stepper', 'robomaster',
                      'rs485_motor', 'encoder', 'imu', 'gps', 'analog_sensor', 'digital_io'},
            'arduino': {'servo', 'dc_motor', 'bldc_motor', 'stepper', 'encoder',
                        'imu', 'gps', 'analog_sensor', 'digital_io'},
            'esp32': {'servo', 'dc_motor', 'bldc_motor', 'stepper', 'encoder',
                      'imu', 'gps', 'analog_sensor', 'digital_io'},
        }

        if platform_type not in platform_capabilities:
            return

        supported_types = platform_capabilities[platform_type]

        for i, device in enumerate(devices):
            device_type = device.get('type')
            device_name = device.get('name', f'device_{i}')

            if device_type not in supported_types:
                self._add_warning(
                    f"Device type '{device_type}' ({device_name}) may not be fully "
                    f"supported on platform '{platform_type}'",
                    f"devices[{i}]",
                    f"Supported types: {', '.join(supported_types)}"
                )

    def print_report(self):
        """Print validation report"""
        if not self.warnings and not self.errors:
            print("✓ No validation issues found")
            return

        if self.errors:
            print(f"\n{'='*60}")
            print(f"ERRORS ({len(self.errors)}):")
            print(f"{'='*60}")
            for i, error in enumerate(self.errors, 1):
                print(f"\n{i}. {error.message}")
                print(f"   Location: {error.location}")
                if error.suggestion:
                    print(f"   Suggestion: {error.suggestion}")

        if self.warnings:
            print(f"\n{'='*60}")
            print(f"WARNINGS ({len(self.warnings)}):")
            print(f"{'='*60}")
            for i, warning in enumerate(self.warnings, 1):
                print(f"\n{i}. {warning.message}")
                print(f"   Location: {warning.location}")
                if warning.suggestion:
                    print(f"   Suggestion: {warning.suggestion}")

        print(f"\n{'='*60}")
        print(f"Summary: {len(self.errors)} errors, {len(self.warnings)} warnings")
        print(f"{'='*60}")


def validate_config(config: Dict[str, Any], verbose: bool = True) -> bool:
    """Validate configuration

    Args:
        config: Configuration dictionary
        verbose: Print validation report

    Returns:
        True if no errors, False if errors found
    """
    validator = ConfigValidators(config)
    warnings, errors = validator.validate_all()

    if verbose:
        validator.print_report()

    return len(errors) == 0


if __name__ == '__main__':
    print("Configuration Validators")
    print("Import this module to use validation functions")
