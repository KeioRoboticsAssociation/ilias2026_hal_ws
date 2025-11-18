#!/usr/bin/env python3
"""
Unified Device Generator

Extends the base MAVLink HAL code generator to support unified device interface.
Generates device creation code using Epic3 unified device factories.

Usage:
    python unified_generator.py <config_file> --platform stm32 --output build/generated
"""

import sys
from pathlib import Path
from typing import Dict, List, Any, Set
from generator import CodeGenerator, GeneratorConfig, GeneratedFile


class UnifiedDeviceGenerator(CodeGenerator):
    """Extended code generator for unified device interface"""

    # Override generated files to include unified device files
    GENERATED_FILES = {
        'stm32': [
            'mavlink_unified_devices.h',
            'mavlink_unified_devices.c',
            'mavlink_unified_handlers.h',
            'mavlink_unified_handlers.c',
        ],
        'arduino': [
            'mavlink_unified_devices.h',
            'mavlink_unified_devices.cpp',
            'mavlink_unified_handlers.h',
            'mavlink_unified_handlers.cpp',
        ],
        'esp32': [
            'mavlink_unified_devices.h',
            'mavlink_unified_devices.c',
            'mavlink_unified_handlers.h',
            'mavlink_unified_handlers.c',
        ],
    }

    # Template mapping (template_name.j2 -> output_name)
    TEMPLATE_MAPPING = {
        'stm32': {
            'unified_devices.h.j2': 'mavlink_unified_devices.h',
            'unified_devices.c.j2': 'mavlink_unified_devices.c',
            'unified_handlers.h.j2': 'mavlink_unified_handlers.h',
            'unified_handlers.c.j2': 'mavlink_unified_handlers.c',
        },
        'arduino': {
            'unified_devices.h.j2': 'mavlink_unified_devices.h',
            'unified_devices.cpp.j2': 'mavlink_unified_devices.cpp',
            'unified_handlers.h.j2': 'mavlink_unified_handlers.h',
            'unified_handlers.cpp.j2': 'mavlink_unified_handlers.cpp',
        },
        'esp32': {
            'unified_devices.h.j2': 'mavlink_unified_devices.h',
            'unified_devices.c.j2': 'mavlink_unified_devices.c',
            'unified_handlers.h.j2': 'mavlink_unified_handlers.h',
            'unified_handlers.c.j2': 'mavlink_unified_handlers.c',
        },
    }

    def _extract_hardware_resources(self, devices: List[Dict]) -> Dict[str, Any]:
        """Extract hardware resource information from devices

        Args:
            devices: List of device configurations

        Returns:
            Dictionary with hardware resource metadata
        """
        timer_ids: Set[int] = set()
        uart_ids: Set[int] = set()
        has_can_devices = False
        has_motor_devices = False

        motor_types = {'servo', 'dc_motor', 'bldc_motor', 'stepper',
                      'robomaster', 'rs485_motor'}

        for device in devices:
            hardware = device.get('hardware', {})
            device_type = device.get('type', '')

            # Extract timer IDs
            if 'timer' in hardware:
                timer_ids.add(hardware['timer'])
            if 'timer_pwm' in hardware:
                timer_ids.add(hardware['timer_pwm'])
            if 'timer_encoder' in hardware:
                timer_ids.add(hardware['timer_encoder'])

            # Extract UART IDs
            if 'uart' in hardware:
                uart_ids.add(hardware['uart'])

            # Check for CAN devices
            if device_type == 'robomaster' or 'can_id' in hardware:
                has_can_devices = True

            # Check for motor devices
            if device_type in motor_types:
                has_motor_devices = True

        return {
            'timer_ids': sorted(list(timer_ids)),
            'uart_ids': sorted(list(uart_ids)),
            'has_can_devices': has_can_devices,
            'has_motor_devices': has_motor_devices,
        }

    def _prepare_template_context(self) -> Dict[str, Any]:
        """Prepare context data for template rendering

        Extends base context with unified device-specific metadata.

        Returns:
            Dictionary with all template variables
        """
        # Get base context from parent class
        context = super()._prepare_template_context()

        # Extract devices
        devices = self.device_config.get('devices', [])

        # Add hardware resource metadata
        hardware_resources = self._extract_hardware_resources(devices)
        context.update(hardware_resources)

        # Add generation_timestamp if not already present
        if 'generation_timestamp' not in context:
            from datetime import datetime
            context['generation_timestamp'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        return context

    def generate(self) -> List[GeneratedFile]:
        """Generate unified device files from configuration

        Returns:
            List of generated files
        """
        # Load schema and config
        self.schema = self._load_schema()
        self.device_config = self._load_config()

        # Validate configuration
        self._validate_config()

        # Prepare template context
        context = self._prepare_template_context()

        if self.config.verbose:
            print(f"Generating unified device code for platform: {self.config.platform}")
            print(f"  Devices: {context['device_count']}")
            print(f"  Timers used: {context['timer_ids']}")
            print(f"  UARTs used: {context['uart_ids']}")
            print(f"  CAN devices: {context['has_can_devices']}")

        # Get template mapping for platform
        template_mapping = self.TEMPLATE_MAPPING.get(self.config.platform, {})

        # Generate each file
        for template_name, output_name in template_mapping.items():
            gen_file = self._generate_file(template_name, output_name, context)
            self.generated_files.append(gen_file)

        if self.config.verbose:
            modified_count = sum(1 for f in self.generated_files if f.modified)
            print(f"\nGenerated {len(self.generated_files)} files ({modified_count} modified)")

        return self.generated_files


def main():
    """Main entry point for unified device generator"""
    import argparse

    parser = argparse.ArgumentParser(
        description='MAVLink HAL Unified Device Code Generator'
    )
    parser.add_argument('config', help='Configuration file (YAML/JSON)')
    parser.add_argument('-p', '--platform', required=True,
                       choices=['stm32', 'arduino', 'esp32'],
                       help='Target platform')
    parser.add_argument('-o', '--output', required=True,
                       help='Output directory')
    parser.add_argument('--dry-run', action='store_true',
                       help='Preview without writing files')
    parser.add_argument('--diff', action='store_true',
                       help='Show diff of changes')
    parser.add_argument('--force', action='store_true',
                       help='Force regeneration (disable incremental mode)')
    parser.add_argument('--validate', action='store_true',
                       help='Run validation before generation')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Verbose output')

    args = parser.parse_args()

    try:
        # Create generator config
        config_file = Path(args.config)
        output_dir = Path(args.output)

        # Find schema and template directories
        script_dir = Path(__file__).parent
        schema_file = script_dir.parent / 'schema' / 'config_schema.json'
        template_dir = script_dir / 'templates'

        gen_config = GeneratorConfig(
            config_file=config_file,
            platform=args.platform,
            output_dir=output_dir,
            schema_file=schema_file,
            template_dir=template_dir,
            dry_run=args.dry_run,
            diff_mode=args.diff,
            incremental=not args.force,
            verbose=args.verbose,
        )

        # Create unified device generator
        generator = UnifiedDeviceGenerator(gen_config)

        # Generate files
        generated_files = generator.generate()

        # Write files (if not dry run)
        if not args.dry_run:
            output_dir.mkdir(parents=True, exist_ok=True)

            for gen_file in generated_files:
                if gen_file.modified or args.force:
                    with open(gen_file.path, 'w') as f:
                        f.write(gen_file.content)
                    if args.verbose:
                        print(f"  Wrote: {gen_file.path}")
                else:
                    if args.verbose:
                        print(f"  Skipped (unchanged): {gen_file.path}")

            print(f"\n✓ Generated {len(generated_files)} unified device files")
            print(f"  Output directory: {output_dir}")
        else:
            print(f"\n✓ Dry run complete ({len(generated_files)} files would be generated)")

    except Exception as e:
        print(f"\n✗ Error: {e}", file=sys.stderr)
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
