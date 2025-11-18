#!/usr/bin/env python3
"""
MAVLink HAL Code Generator

Generates platform-specific C/C++ code from YAML/JSON configuration files.
Supports STM32, Arduino, ESP32 platforms with optimized code generation.

Usage:
    python generator.py <config_file> --platform stm32 --output build/generated
"""

import json
import yaml
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime
import hashlib

try:
    from jinja2 import Environment, FileSystemLoader, Template
    import jsonschema
except ImportError:
    print("Error: Required packages not installed")
    print("Run: pip install jinja2 jsonschema pyyaml")
    exit(1)


@dataclass
class GeneratorConfig:
    """Configuration for code generator"""
    config_file: Path
    platform: str
    output_dir: Path
    schema_file: Path
    template_dir: Path
    dry_run: bool = False
    diff_mode: bool = False
    incremental: bool = True
    optimize: bool = True
    verbose: bool = False


@dataclass
class GeneratedFile:
    """Represents a generated file"""
    path: Path
    content: str
    checksum: str
    modified: bool


class CodeGenerator:
    """Main code generator class"""

    SUPPORTED_PLATFORMS = ['stm32', 'arduino', 'esp32', 'linux']

    GENERATED_FILES = {
        'stm32': [
            'mavlink_generated_config.h',
            'mavlink_generated_devices.c',
            'mavlink_generated_handlers.c',
            'mavlink_generated_params.h',
        ],
        'arduino': [
            'mavlink_generated_config.h',
            'mavlink_generated_devices.cpp',
            'mavlink_generated_handlers.cpp',
            'mavlink_generated_params.h',
        ],
        'esp32': [
            'mavlink_generated_config.h',
            'mavlink_generated_devices.c',
            'mavlink_generated_handlers.c',
            'mavlink_generated_params.h',
        ],
    }

    def __init__(self, config: GeneratorConfig):
        """Initialize code generator

        Args:
            config: Generator configuration
        """
        self.config = config
        self.jinja_env: Optional[Environment] = None
        self.schema: Optional[Dict] = None
        self.device_config: Optional[Dict] = None
        self.generated_files: List[GeneratedFile] = []

        # Initialize Jinja2 environment
        self._init_jinja()

    def _init_jinja(self):
        """Initialize Jinja2 template environment"""
        template_dir = self.config.template_dir / self.config.platform

        if not template_dir.exists():
            raise FileNotFoundError(
                f"Template directory not found: {template_dir}\n"
                f"Platform '{self.config.platform}' may not be supported yet."
            )

        self.jinja_env = Environment(
            loader=FileSystemLoader(template_dir),
            trim_blocks=True,
            lstrip_blocks=True,
            keep_trailing_newline=True
        )

        # Add custom filters
        self.jinja_env.filters['to_c_identifier'] = self._to_c_identifier
        self.jinja_env.filters['to_upper'] = lambda s: s.upper()
        self.jinja_env.filters['to_hex'] = lambda n: f"0x{n:X}"
        self.jinja_env.filters['format_float'] = lambda f: f"{f:.6f}f"
        self.jinja_env.filters['uart_to_number'] = self._uart_to_number

    def _to_c_identifier(self, s: str) -> str:
        """Convert string to valid C identifier"""
        return s.replace('-', '_').replace(' ', '_').replace('.', '_')

    def _uart_to_number(self, uart_name: str) -> int:
        """Extract UART number from name (e.g., 'USART1' -> 1, 'UART2' -> 2)"""
        import re
        match = re.search(r'(\d+)', str(uart_name))
        return int(match.group(1)) if match else 1

    def _load_schema(self) -> Dict:
        """Load and parse JSON schema"""
        if self.config.verbose:
            print(f"Loading schema: {self.config.schema_file}")

        try:
            with open(self.config.schema_file, 'r') as f:
                schema = json.load(f)
            return schema
        except FileNotFoundError:
            raise FileNotFoundError(f"Schema file not found: {self.config.schema_file}")
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in schema file: {e}")

    def _load_config(self) -> Dict:
        """Load and parse configuration file (YAML or JSON)"""
        if self.config.verbose:
            print(f"Loading config: {self.config.config_file}")

        try:
            with open(self.config.config_file, 'r') as f:
                content = f.read()

            # Try YAML first (more permissive)
            try:
                config = yaml.safe_load(content)
                if self.config.verbose:
                    print("  Format: YAML")
                return config
            except yaml.YAMLError:
                # Try JSON
                try:
                    config = json.loads(content)
                    if self.config.verbose:
                        print("  Format: JSON")
                    return config
                except json.JSONDecodeError as e:
                    raise ValueError(f"Invalid YAML/JSON in config file: {e}")

        except FileNotFoundError:
            raise FileNotFoundError(f"Config file not found: {self.config.config_file}")

    def _validate_config(self) -> bool:
        """Validate configuration against schema

        Returns:
            True if valid, raises exception otherwise
        """
        if self.config.verbose:
            print("Validating configuration...")

        try:
            jsonschema.validate(
                instance=self.device_config,
                schema=self.schema
            )
            if self.config.verbose:
                print("  ✓ Configuration is valid")
            return True

        except jsonschema.ValidationError as e:
            error_path = " → ".join(str(p) for p in e.path)
            raise ValueError(
                f"Configuration validation failed:\n"
                f"  Location: {error_path}\n"
                f"  Error: {e.message}\n"
                f"  Expected: {e.validator_value if e.validator_value else 'see schema'}"
            )

    def _calculate_checksum(self, content: str) -> str:
        """Calculate SHA256 checksum of content"""
        return hashlib.sha256(content.encode('utf-8')).hexdigest()[:16]

    def _should_generate(self, output_file: Path, new_checksum: str) -> bool:
        """Check if file should be generated (incremental mode)

        Args:
            output_file: Output file path
            new_checksum: Checksum of new content

        Returns:
            True if file should be generated
        """
        if not self.config.incremental:
            return True

        if not output_file.exists():
            return True

        # Check for checksum in existing file
        try:
            with open(output_file, 'r') as f:
                content = f.read()
                # Look for checksum in header comment
                for line in content.split('\n')[:20]:  # Check first 20 lines
                    if 'Checksum:' in line:
                        old_checksum = line.split('Checksum:')[1].strip()
                        return old_checksum != new_checksum
            # No checksum found, regenerate
            return True
        except Exception:
            return True

    def _prepare_template_context(self) -> Dict[str, Any]:
        """Prepare context data for template rendering

        Returns:
            Dictionary with all template variables
        """
        # Extract key information
        platform_info = self.device_config.get('platform', {})
        mavlink_info = self.device_config.get('mavlink', {})
        devices = self.device_config.get('devices', [])
        system_info = self.device_config.get('system', {})

        # Group devices by type
        devices_by_type = {}
        for device in devices:
            device_type = device.get('type')
            if device_type not in devices_by_type:
                devices_by_type[device_type] = []
            devices_by_type[device_type].append(device)

        # Create context
        context = {
            'config': self.device_config,
            'platform': platform_info,
            'mavlink': mavlink_info,
            'devices': devices,
            'devices_by_type': devices_by_type,
            'system': system_info,

            # Metadata
            'generated_timestamp': datetime.now().isoformat(),
            'config_file': str(self.config.config_file.name),
            'generator_version': '1.0.0',

            # Counts
            'device_count': len(devices),
            'servo_count': len(devices_by_type.get('servo', [])),
            'dc_motor_count': len(devices_by_type.get('dc_motor', [])),
            'bldc_motor_count': len(devices_by_type.get('bldc_motor', [])),
            'stepper_count': len(devices_by_type.get('stepper', [])),
            'robomaster_count': len(devices_by_type.get('robomaster', [])),
            'rs485_motor_count': len(devices_by_type.get('rs485_motor', [])),
            'encoder_count': len(devices_by_type.get('encoder', [])),
            'imu_count': len(devices_by_type.get('imu', [])),
            'gps_count': len(devices_by_type.get('gps', [])),
            'analog_sensor_count': len(devices_by_type.get('analog_sensor', [])),
            'digital_io_count': len(devices_by_type.get('digital_io', [])),
        }

        return context

    def _render_template(self, template_name: str, context: Dict[str, Any]) -> str:
        """Render a Jinja2 template

        Args:
            template_name: Template file name
            context: Template context variables

        Returns:
            Rendered template content
        """
        try:
            template = self.jinja_env.get_template(template_name)
            content = template.render(**context)
            return content
        except Exception as e:
            raise RuntimeError(f"Template rendering failed for {template_name}: {e}")

    def _generate_file(self, template_name: str, output_name: str,
                       context: Dict[str, Any]) -> GeneratedFile:
        """Generate a single file from template

        Args:
            template_name: Template file name
            output_name: Output file name
            context: Template context

        Returns:
            GeneratedFile object
        """
        if self.config.verbose:
            print(f"  Generating: {output_name}")

        # Render template
        content = self._render_template(template_name, context)

        # Add checksum to content (in header comment)
        checksum = self._calculate_checksum(content)

        # Insert checksum into header (after first comment block)
        lines = content.split('\n')
        for i, line in enumerate(lines[:10]):
            if ' * Generated:' in line or '// Generated:' in line:
                # Insert checksum after generation timestamp
                comment_prefix = ' *' if ' *' in line else '//'
                lines.insert(i + 1, f"{comment_prefix} Checksum: {checksum}")
                content = '\n'.join(lines)
                break

        # Create output path
        output_path = self.config.output_dir / output_name

        # Check if should generate (incremental mode)
        modified = self._should_generate(output_path, checksum)

        return GeneratedFile(
            path=output_path,
            content=content,
            checksum=checksum,
            modified=modified
        )

    def generate(self) -> List[GeneratedFile]:
        """Generate all files from configuration

        Returns:
            List of generated files
        """
        # Load schema and config
        self.schema = self._load_schema()
        self.device_config = self._load_config()

        # Validate configuration
        self._validate_config()

        # Verify platform matches
        config_platform = self.device_config.get('platform', {}).get('type')
        if config_platform != self.config.platform:
            print(f"Warning: Config platform '{config_platform}' differs from "
                  f"generator platform '{self.config.platform}'")

        # Prepare template context
        context = self._prepare_template_context()

        # Get list of files to generate for this platform
        file_list = self.GENERATED_FILES.get(self.config.platform, [])

        if not file_list:
            raise ValueError(f"No file list defined for platform: {self.config.platform}")

        print(f"\nGenerating code for platform: {self.config.platform}")
        print(f"Output directory: {self.config.output_dir}")
        print(f"Files to generate: {len(file_list)}")

        # Generate each file
        self.generated_files = []

        template_map = {
            'mavlink_generated_config.h': 'config.h.j2',
            'mavlink_generated_devices.c': 'devices.c.j2',
            'mavlink_generated_devices.cpp': 'devices.cpp.j2',
            'mavlink_generated_handlers.c': 'handlers.c.j2',
            'mavlink_generated_handlers.cpp': 'handlers.cpp.j2',
            'mavlink_generated_params.h': 'params.h.j2',
        }

        for output_name in file_list:
            template_name = template_map.get(output_name)
            if not template_name:
                print(f"Warning: No template mapping for {output_name}, skipping")
                continue

            try:
                generated_file = self._generate_file(template_name, output_name, context)
                self.generated_files.append(generated_file)
            except Exception as e:
                print(f"Error generating {output_name}: {e}")
                if self.config.verbose:
                    import traceback
                    traceback.print_exc()
                raise

        return self.generated_files

    def write_files(self):
        """Write generated files to disk"""
        if not self.generated_files:
            print("No files to write")
            return

        # Create output directory if needed
        self.config.output_dir.mkdir(parents=True, exist_ok=True)

        modified_count = 0
        skipped_count = 0

        for gen_file in self.generated_files:
            if self.config.dry_run:
                status = "WOULD CREATE" if not gen_file.path.exists() else "WOULD UPDATE"
                print(f"  [{status}] {gen_file.path.name}")
                continue

            if not gen_file.modified and self.config.incremental:
                if self.config.verbose:
                    print(f"  [SKIP] {gen_file.path.name} (unchanged)")
                skipped_count += 1
                continue

            # Write file
            with open(gen_file.path, 'w') as f:
                f.write(gen_file.content)

            status = "CREATE" if not gen_file.path.exists() else "UPDATE"
            print(f"  ✓ {gen_file.path.name} ({len(gen_file.content)} bytes)")
            modified_count += 1

        # Summary
        print(f"\nGeneration complete:")
        print(f"  Modified: {modified_count}")
        print(f"  Skipped: {skipped_count}")
        print(f"  Total: {len(self.generated_files)}")

        if self.config.dry_run:
            print("\n(Dry run mode - no files were written)")

    def show_diff(self):
        """Show diff of changes (diff mode)"""
        if not self.config.diff_mode:
            return

        print("\nDiff mode - showing changes:")

        for gen_file in self.generated_files:
            if not gen_file.path.exists():
                print(f"\n[NEW FILE] {gen_file.path.name}")
                print("  (Would create new file)")
                continue

            # Read existing content
            with open(gen_file.path, 'r') as f:
                old_content = f.read()

            if old_content == gen_file.content:
                print(f"\n[UNCHANGED] {gen_file.path.name}")
                continue

            print(f"\n[MODIFIED] {gen_file.path.name}")

            # Simple line-by-line diff
            old_lines = old_content.split('\n')
            new_lines = gen_file.content.split('\n')

            # Show first few differences
            diff_count = 0
            for i, (old_line, new_line) in enumerate(zip(old_lines, new_lines)):
                if old_line != new_line and diff_count < 5:
                    print(f"  Line {i+1}:")
                    print(f"    - {old_line[:80]}")
                    print(f"    + {new_line[:80]}")
                    diff_count += 1

            if diff_count >= 5:
                print(f"  ... ({len(new_lines) - len(old_lines)} more changes)")


def create_generator(config_file: str, platform: str, output_dir: str,
                     **kwargs) -> CodeGenerator:
    """Factory function to create code generator

    Args:
        config_file: Path to configuration file
        platform: Target platform (stm32, arduino, esp32)
        output_dir: Output directory for generated files
        **kwargs: Additional generator options

    Returns:
        CodeGenerator instance
    """
    # Resolve paths
    config_path = Path(config_file).resolve()
    output_path = Path(output_dir).resolve()

    # Find schema and template directories
    script_dir = Path(__file__).parent
    schema_path = script_dir.parent / 'schemas' / 'device_config.schema.json'
    template_path = script_dir / 'templates'

    # Create generator config
    gen_config = GeneratorConfig(
        config_file=config_path,
        platform=platform,
        output_dir=output_path,
        schema_file=schema_path,
        template_dir=template_path,
        dry_run=kwargs.get('dry_run', False),
        diff_mode=kwargs.get('diff_mode', False),
        incremental=kwargs.get('incremental', True),
        optimize=kwargs.get('optimize', True),
        verbose=kwargs.get('verbose', False)
    )

    return CodeGenerator(gen_config)


if __name__ == '__main__':
    print("MAVLink HAL Code Generator")
    print("Use cli.py for command-line interface")
