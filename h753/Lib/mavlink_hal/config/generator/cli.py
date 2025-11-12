#!/usr/bin/env python3
"""
MAVLink HAL Code Generator - Command Line Interface

Usage:
    python cli.py generate <config_file> --platform stm32 --output build/generated
    python cli.py validate <config_file>
    python cli.py clean <output_dir>
"""

import argparse
import sys
from pathlib import Path
from typing import Optional

try:
    from generator import create_generator, CodeGenerator
    from validators import ConfigValidators
    import yaml
    import json
except ImportError as e:
    print(f"Error: Required module not found: {e}")
    print("Run: pip install -r requirements.txt")
    sys.exit(1)


def cmd_generate(args):
    """Generate code from configuration file"""
    try:
        # Create generator
        generator = create_generator(
            config_file=args.config,
            platform=args.platform,
            output_dir=args.output,
            dry_run=args.dry_run,
            diff_mode=args.diff,
            incremental=not args.force,
            optimize=not args.no_optimize,
            verbose=args.verbose
        )

        # Run additional validation if requested
        if args.validate:
            print("Running additional validation checks...")
            config = generator.device_config
            validator = ConfigValidators(config)
            warnings, errors = validator.validate_all()

            if errors or warnings:
                validator.print_report()

            if errors:
                print("\n✗ Validation failed with errors. Aborting generation.")
                return 1

            if warnings and not args.ignore_warnings:
                response = input("\nWarnings found. Continue anyway? (y/N): ")
                if response.lower() != 'y':
                    print("Aborted by user.")
                    return 1

        # Generate files
        print("")
        generated_files = generator.generate()

        # Show diff if requested
        if args.diff:
            generator.show_diff()

        # Write files
        if not args.dry_run:
            generator.write_files()

        print("\n✓ Code generation completed successfully!")
        return 0

    except FileNotFoundError as e:
        print(f"✗ Error: {e}")
        return 1
    except ValueError as e:
        print(f"✗ Validation Error: {e}")
        return 1
    except Exception as e:
        print(f"✗ Error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


def cmd_validate(args):
    """Validate configuration file"""
    try:
        config_path = Path(args.config).resolve()

        if not config_path.exists():
            print(f"✗ Error: Config file not found: {config_path}")
            return 1

        print(f"Validating: {config_path.name}")
        print("=" * 60)

        # Load configuration
        with open(config_path, 'r') as f:
            content = f.read()

        try:
            config = yaml.safe_load(content)
            print("Format: YAML")
        except yaml.YAMLError:
            try:
                config = json.loads(content)
                print("Format: JSON")
            except json.JSONDecodeError as e:
                print(f"✗ Invalid YAML/JSON: {e}")
                return 1

        # Basic schema validation
        script_dir = Path(__file__).parent
        schema_path = script_dir.parent / 'schemas' / 'device_config.schema.json'

        if schema_path.exists():
            import jsonschema

            with open(schema_path, 'r') as f:
                schema = json.load(f)

            try:
                jsonschema.validate(instance=config, schema=schema)
                print("✓ JSON Schema validation passed")
            except jsonschema.ValidationError as e:
                print(f"✗ Schema Validation Error:")
                print(f"  Location: {' → '.join(str(p) for p in e.path)}")
                print(f"  Error: {e.message}")
                return 1

        # Additional validation
        print("\nRunning additional validation checks...")
        validator = ConfigValidators(config)
        warnings, errors = validator.validate_all()

        validator.print_report()

        if errors:
            print("\n✗ Validation failed")
            return 1
        else:
            print("\n✓ Validation passed")
            if warnings:
                print(f"  (with {len(warnings)} warnings)")
            return 0

    except Exception as e:
        print(f"✗ Error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


def cmd_clean(args):
    """Clean generated files"""
    try:
        output_dir = Path(args.output).resolve()

        if not output_dir.exists():
            print(f"Output directory does not exist: {output_dir}")
            return 0

        # List of generated files to remove
        generated_patterns = [
            'mavlink_generated_*.h',
            'mavlink_generated_*.c',
            'mavlink_generated_*.cpp',
        ]

        import glob

        removed_count = 0
        for pattern in generated_patterns:
            for file_path in output_dir.glob(pattern):
                if args.dry_run:
                    print(f"Would remove: {file_path.name}")
                else:
                    file_path.unlink()
                    print(f"Removed: {file_path.name}")
                removed_count += 1

        if removed_count == 0:
            print("No generated files found to remove")
        else:
            if args.dry_run:
                print(f"\nWould remove {removed_count} files (dry run)")
            else:
                print(f"\n✓ Removed {removed_count} generated files")

        return 0

    except Exception as e:
        print(f"✗ Error: {e}")
        return 1


def cmd_list_platforms(args):
    """List supported platforms"""
    from generator import CodeGenerator

    print("Supported platforms:")
    for platform in CodeGenerator.SUPPORTED_PLATFORMS:
        print(f"  - {platform}")

    return 0


def cmd_info(args):
    """Show configuration information"""
    try:
        config_path = Path(args.config).resolve()

        if not config_path.exists():
            print(f"✗ Error: Config file not found: {config_path}")
            return 1

        # Load configuration
        with open(config_path, 'r') as f:
            content = f.read()

        try:
            config = yaml.safe_load(content)
        except yaml.YAMLError:
            config = json.loads(content)

        # Display info
        print(f"\nConfiguration: {config_path.name}")
        print("=" * 60)

        platform = config.get('platform', {})
        print(f"\nPlatform: {platform.get('type', 'Unknown')}")
        print(f"  MCU: {platform.get('mcu', {}).get('model', 'Unknown')}")
        print(f"  Clock: {platform.get('mcu', {}).get('clock_mhz', '?')} MHz")

        mavlink = config.get('mavlink', {})
        transport = mavlink.get('transport', {})
        print(f"\nMAVLink:")
        print(f"  System ID: {mavlink.get('system_id', '?')}")
        print(f"  Transport: {transport.get('type', 'Unknown')}")

        devices = config.get('devices', [])
        print(f"\nDevices: {len(devices)}")

        device_types = {}
        for device in devices:
            dtype = device.get('type', 'unknown')
            device_types[dtype] = device_types.get(dtype, 0) + 1

        for dtype, count in sorted(device_types.items()):
            print(f"  {dtype}: {count}")

        print("")
        return 0

    except Exception as e:
        print(f"✗ Error: {e}")
        return 1


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='MAVLink HAL Code Generator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate code for STM32 platform
  python cli.py generate config.yaml --platform stm32 --output build/generated

  # Validate configuration
  python cli.py validate config.yaml

  # Show configuration info
  python cli.py info config.yaml

  # Clean generated files
  python cli.py clean build/generated

  # List supported platforms
  python cli.py platforms
"""
    )

    subparsers = parser.add_subparsers(dest='command', help='Command to execute')

    # Generate command
    gen_parser = subparsers.add_parser('generate', help='Generate code from configuration')
    gen_parser.add_argument('config', help='Configuration file (YAML or JSON)')
    gen_parser.add_argument('--platform', '-p', required=True,
                            choices=['stm32', 'arduino', 'esp32', 'linux'],
                            help='Target platform')
    gen_parser.add_argument('--output', '-o', required=True,
                            help='Output directory for generated files')
    gen_parser.add_argument('--dry-run', '-n', action='store_true',
                            help='Show what would be generated without writing files')
    gen_parser.add_argument('--diff', '-d', action='store_true',
                            help='Show diff of changes')
    gen_parser.add_argument('--force', '-f', action='store_true',
                            help='Force regeneration (disable incremental mode)')
    gen_parser.add_argument('--no-optimize', action='store_true',
                            help='Disable code optimization')
    gen_parser.add_argument('--validate', '-V', action='store_true',
                            help='Run additional validation before generation')
    gen_parser.add_argument('--ignore-warnings', '-w', action='store_true',
                            help='Ignore warnings during validation')
    gen_parser.add_argument('--verbose', '-v', action='store_true',
                            help='Verbose output')

    # Validate command
    val_parser = subparsers.add_parser('validate', help='Validate configuration file')
    val_parser.add_argument('config', help='Configuration file to validate')
    val_parser.add_argument('--verbose', '-v', action='store_true',
                            help='Verbose output')

    # Clean command
    clean_parser = subparsers.add_parser('clean', help='Clean generated files')
    clean_parser.add_argument('output', help='Output directory to clean')
    clean_parser.add_argument('--dry-run', '-n', action='store_true',
                              help='Show what would be removed without deleting')

    # Info command
    info_parser = subparsers.add_parser('info', help='Show configuration information')
    info_parser.add_argument('config', help='Configuration file')

    # List platforms command
    subparsers.add_parser('platforms', help='List supported platforms')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return 1

    # Dispatch to command handler
    if args.command == 'generate':
        return cmd_generate(args)
    elif args.command == 'validate':
        return cmd_validate(args)
    elif args.command == 'clean':
        return cmd_clean(args)
    elif args.command == 'platforms':
        return cmd_list_platforms(args)
    elif args.command == 'info':
        return cmd_info(args)
    else:
        parser.print_help()
        return 1


if __name__ == '__main__':
    sys.exit(main())
