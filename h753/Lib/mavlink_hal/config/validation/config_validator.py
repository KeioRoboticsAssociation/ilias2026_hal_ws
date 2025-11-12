#!/usr/bin/env python3
"""
MAVLink Device Configuration Validator

Validates YAML/JSON configuration files against the device_config schema.
Supports JSON Schema Draft-07 with custom validation rules.

Usage:
    python3 config_validator.py <config_file>
    python3 config_validator.py --all  # Validate all examples
    python3 config_validator.py --schema  # Validate schema itself

Requirements:
    pip install jsonschema pyyaml
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Any, Tuple

try:
    import jsonschema
    from jsonschema import validate, Draft7Validator
    import yaml
except ImportError:
    print("Error: Required packages not installed")
    print("Run: pip install jsonschema pyyaml")
    sys.exit(1)


class ConfigValidator:
    """Validator for MAVLink device configurations"""

    def __init__(self, schema_path: Path):
        """Initialize validator with schema file

        Args:
            schema_path: Path to JSON schema file
        """
        self.schema_path = schema_path
        self.schema = self._load_schema()
        self.validator = Draft7Validator(self.schema)

    def _load_schema(self) -> Dict:
        """Load and parse JSON schema"""
        try:
            with open(self.schema_path, 'r') as f:
                schema = json.load(f)
            return schema
        except FileNotFoundError:
            print(f"Error: Schema file not found: {self.schema_path}")
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON in schema file: {e}")
            sys.exit(1)

    def _load_config(self, config_path: Path) -> Tuple[Dict, str]:
        """Load configuration file (YAML or JSON)

        Returns:
            Tuple of (config_dict, format_type)
        """
        try:
            with open(config_path, 'r') as f:
                content = f.read()

            # Try YAML first (more permissive)
            try:
                config = yaml.safe_load(content)
                return config, 'yaml'
            except yaml.YAMLError:
                # Try JSON
                try:
                    config = json.loads(content)
                    return config, 'json'
                except json.JSONDecodeError as e:
                    print(f"Error: Invalid YAML/JSON in config file: {e}")
                    return None, None

        except FileNotFoundError:
            print(f"Error: Config file not found: {config_path}")
            return None, None

    def validate_config(self, config_path: Path, verbose: bool = True) -> bool:
        """Validate a configuration file

        Args:
            config_path: Path to configuration file
            verbose: Print detailed validation results

        Returns:
            True if valid, False otherwise
        """
        if verbose:
            print(f"\nValidating: {config_path.name}")
            print("=" * 60)

        config, fmt = self._load_config(config_path)
        if config is None:
            return False

        if verbose:
            print(f"Format: {fmt.upper()}")

        # Validate against schema
        errors = list(self.validator.iter_errors(config))

        if not errors:
            if verbose:
                print("✓ Configuration is valid!")
                self._print_summary(config)
            return True
        else:
            if verbose:
                print("✗ Configuration has errors:")
                for i, error in enumerate(errors, 1):
                    self._print_error(i, error)
            return False

    def _print_summary(self, config: Dict):
        """Print configuration summary"""
        print("\nConfiguration Summary:")
        print(f"  Platform: {config.get('platform', {}).get('type', 'Unknown')}")
        print(f"  MCU: {config.get('platform', {}).get('mcu', {}).get('model', 'Unknown')}")

        transport = config.get('mavlink', {}).get('transport', {})
        transport_type = transport.get('type', 'Unknown')
        print(f"  MAVLink Transport: {transport_type}")

        if transport_type == 'uart':
            uart = transport.get('uart', {})
            print(f"    Port: {uart.get('port', 'N/A')}")
            print(f"    Baudrate: {uart.get('baudrate', 'N/A')}")
        elif transport_type == 'udp':
            udp = transport.get('udp', {})
            print(f"    Local: {udp.get('local_ip', 'N/A')}:{udp.get('local_port', 'N/A')}")
            print(f"    Remote: {udp.get('remote_ip', 'N/A')}:{udp.get('remote_port', 'N/A')}")

        devices = config.get('devices', [])
        print(f"  Devices: {len(devices)}")

        device_types = {}
        for device in devices:
            dtype = device.get('type', 'unknown')
            device_types[dtype] = device_types.get(dtype, 0) + 1

        for dtype, count in sorted(device_types.items()):
            print(f"    {dtype}: {count}")

    def _print_error(self, num: int, error: jsonschema.ValidationError):
        """Print formatted validation error"""
        print(f"\n  {num}. {error.message}")

        if error.path:
            path = " → ".join(str(p) for p in error.path)
            print(f"     Location: {path}")

        if error.schema_path:
            schema_path = " → ".join(str(p) for p in error.schema_path)
            print(f"     Schema: {schema_path}")

        if error.validator_value and error.validator != 'required':
            print(f"     Expected: {error.validator_value}")

    def validate_schema(self) -> bool:
        """Validate the schema itself"""
        print("\nValidating schema file...")
        print("=" * 60)

        try:
            jsonschema.Draft7Validator.check_schema(self.schema)
            print("✓ Schema is valid JSON Schema Draft-07!")

            # Print schema info
            print(f"\nSchema Info:")
            print(f"  Title: {self.schema.get('title', 'N/A')}")
            print(f"  Version: {self.schema.get('version', 'N/A')}")
            print(f"  Description: {self.schema.get('description', 'N/A')}")

            # Count definitions
            definitions = self.schema.get('definitions', {})
            print(f"  Definitions: {len(definitions)}")
            for name in sorted(definitions.keys()):
                print(f"    - {name}")

            return True

        except jsonschema.SchemaError as e:
            print(f"✗ Schema is invalid: {e.message}")
            return False


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Validate MAVLink device configuration files'
    )
    parser.add_argument(
        'config_file',
        nargs='?',
        help='Configuration file to validate (YAML or JSON)'
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='Validate all example configurations'
    )
    parser.add_argument(
        '--schema',
        action='store_true',
        help='Validate the schema file itself'
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Only print errors'
    )

    args = parser.parse_args()

    # Find schema file
    script_dir = Path(__file__).parent
    schema_path = script_dir.parent / 'schemas' / 'device_config.schema.json'

    if not schema_path.exists():
        print(f"Error: Schema file not found at {schema_path}")
        sys.exit(1)

    validator = ConfigValidator(schema_path)

    # Validate schema itself
    if args.schema:
        success = validator.validate_schema()
        sys.exit(0 if success else 1)

    # Validate all examples
    if args.all:
        examples_dir = script_dir.parent / 'examples'
        if not examples_dir.exists():
            print(f"Error: Examples directory not found: {examples_dir}")
            sys.exit(1)

        config_files = list(examples_dir.glob('*.yaml')) + list(examples_dir.glob('*.json'))

        if not config_files:
            print(f"No configuration files found in {examples_dir}")
            sys.exit(1)

        print(f"Found {len(config_files)} configuration files")

        results = []
        for config_file in sorted(config_files):
            success = validator.validate_config(config_file, verbose=not args.quiet)
            results.append((config_file.name, success))

        print("\n" + "=" * 60)
        print("Summary:")
        print("=" * 60)

        passed = sum(1 for _, success in results if success)
        failed = len(results) - passed

        for name, success in results:
            status = "✓ PASS" if success else "✗ FAIL"
            print(f"  {status}: {name}")

        print(f"\nTotal: {passed} passed, {failed} failed")
        sys.exit(0 if failed == 0 else 1)

    # Validate single file
    if args.config_file:
        config_path = Path(args.config_file)
        success = validator.validate_config(config_path, verbose=not args.quiet)
        sys.exit(0 if success else 1)

    # No arguments provided
    parser.print_help()
    sys.exit(1)


if __name__ == '__main__':
    main()
