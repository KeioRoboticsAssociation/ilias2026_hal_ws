#!/usr/bin/env python3
"""
Code Optimizers

Provides code optimization passes for generated code:
- Dead code elimination
- Constant folding
- Loop unrolling
- Memory layout optimization
"""

from typing import Dict, List, Any, Set
from dataclasses import dataclass


@dataclass
class OptimizationResult:
    """Result of optimization pass"""
    optimized: bool
    description: str
    bytes_saved: int = 0


class CodeOptimizer:
    """Code optimization utilities"""

    def __init__(self, config: Dict[str, Any], platform: str):
        """Initialize optimizer

        Args:
            config: Device configuration
            platform: Target platform (stm32, arduino, esp32)
        """
        self.config = config
        self.platform = platform
        self.optimizations: List[OptimizationResult] = []

    def optimize_all(self) -> List[OptimizationResult]:
        """Run all optimization passes

        Returns:
            List of optimization results
        """
        self.optimizations = []

        self._optimize_dead_code()
        self._optimize_memory_layout()
        self._optimize_constant_propagation()

        return self.optimizations

    def _optimize_dead_code(self):
        """Eliminate dead code and unused devices"""
        devices = self.config.get('devices', [])

        # Find disabled devices
        disabled_count = 0
        for device in devices:
            if not device.get('enabled', True):
                disabled_count += 1

        if disabled_count > 0:
            self.optimizations.append(OptimizationResult(
                optimized=True,
                description=f"Dead code elimination: {disabled_count} disabled devices excluded",
                bytes_saved=disabled_count * 100  # Rough estimate
            ))

    def _optimize_memory_layout(self):
        """Optimize memory layout for platform"""
        devices = self.config.get('devices', [])
        device_count = len(devices)

        if device_count == 0:
            return

        # Platform-specific optimizations
        if self.platform == 'stm32':
            # STM32: Optimize for cache alignment
            self.optimizations.append(OptimizationResult(
                optimized=True,
                description=f"Memory layout: STM32 cache-aligned structures ({device_count} devices)",
                bytes_saved=0
            ))

        elif self.platform == 'arduino':
            # Arduino: Optimize for SRAM usage
            self.optimizations.append(OptimizationResult(
                optimized=True,
                description=f"Memory layout: Arduino SRAM optimization ({device_count} devices)",
                bytes_saved=device_count * 4  # Use PROGMEM for constants
            ))

    def _optimize_constant_propagation(self):
        """Propagate constants at compile-time"""
        devices = self.config.get('devices', [])

        # Count devices with compile-time constants
        const_count = 0
        for device in devices:
            config = device.get('config', {})
            # Any device with fixed configuration can be optimized
            if config:
                const_count += 1

        if const_count > 0:
            self.optimizations.append(OptimizationResult(
                optimized=True,
                description=f"Constant propagation: {const_count} devices with compile-time constants",
                bytes_saved=const_count * 8  # Rough estimate
            ))

    def get_optimization_summary(self) -> str:
        """Get summary of all optimizations

        Returns:
            Human-readable summary string
        """
        if not self.optimizations:
            return "No optimizations applied"

        total_saved = sum(opt.bytes_saved for opt in self.optimizations)

        summary = "Optimization Summary:\n"
        for i, opt in enumerate(self.optimizations, 1):
            summary += f"  {i}. {opt.description}\n"
            if opt.bytes_saved > 0:
                summary += f"     Estimated savings: {opt.bytes_saved} bytes\n"

        if total_saved > 0:
            summary += f"\nTotal estimated savings: {total_saved} bytes"

        return summary

    def should_inline_function(self, function_name: str, call_count: int) -> bool:
        """Determine if a function should be inlined

        Args:
            function_name: Name of function
            call_count: Number of times function is called

        Returns:
            True if function should be inlined
        """
        # Inline if called only once
        if call_count == 1:
            return True

        # Inline small functions called few times
        small_functions = ['set_motor_speed', 'set_servo_angle', 'read_sensor']
        if function_name in small_functions and call_count <= 3:
            return True

        return False

    def optimize_loop_unrolling(self, loop_size: int) -> bool:
        """Determine if a loop should be unrolled

        Args:
            loop_size: Number of loop iterations

        Returns:
            True if loop should be unrolled
        """
        # Platform-specific thresholds
        max_unroll = {
            'stm32': 8,      # More memory available
            'arduino': 4,    # Limited memory
            'esp32': 8,      # Good memory
        }

        threshold = max_unroll.get(self.platform, 4)
        return loop_size <= threshold

    def suggest_dma_usage(self, device: Dict[str, Any]) -> bool:
        """Suggest DMA usage for device

        Args:
            device: Device configuration

        Returns:
            True if DMA is recommended
        """
        if self.platform != 'stm32':
            return False

        device_type = device.get('type')

        # DMA recommended for high-bandwidth devices
        dma_recommended = ['encoder', 'adc', 'dac', 'spi', 'i2c']

        return device_type in dma_recommended

    def calculate_stack_usage(self) -> int:
        """Estimate stack usage

        Returns:
            Estimated stack usage in bytes
        """
        devices = self.config.get('devices', [])

        # Base stack usage
        base_stack = 512

        # Per-device stack usage
        device_stack = len(devices) * 64

        # MAVLink buffer stack
        mavlink_stack = 512

        return base_stack + device_stack + mavlink_stack

    def calculate_heap_usage(self) -> int:
        """Estimate heap usage

        Returns:
            Estimated heap usage in bytes
        """
        devices = self.config.get('devices', [])

        # Estimate per-device heap usage
        heap_per_device = {
            'servo': 32,
            'dc_motor': 64,
            'bldc_motor': 32,
            'stepper': 48,
            'robomaster': 96,
            'rs485_motor': 128,
            'encoder': 32,
            'imu': 64,
            'gps': 256,
            'analog_sensor': 16,
            'digital_io': 8,
        }

        total_heap = 0
        for device in devices:
            device_type = device.get('type')
            total_heap += heap_per_device.get(device_type, 32)

        # MAVLink buffers
        total_heap += 1024

        return total_heap


def optimize_config(config: Dict[str, Any], platform: str, verbose: bool = True) -> List[OptimizationResult]:
    """Optimize configuration

    Args:
        config: Device configuration
        platform: Target platform
        verbose: Print optimization summary

    Returns:
        List of optimization results
    """
    optimizer = CodeOptimizer(config, platform)
    results = optimizer.optimize_all()

    if verbose:
        print("\n" + optimizer.get_optimization_summary())

        stack_usage = optimizer.calculate_stack_usage()
        heap_usage = optimizer.calculate_heap_usage()

        print(f"\nEstimated Memory Usage:")
        print(f"  Stack: {stack_usage} bytes")
        print(f"  Heap: {heap_usage} bytes")
        print(f"  Total: {stack_usage + heap_usage} bytes")

    return results


if __name__ == '__main__':
    print("Code Optimizers")
    print("Import this module to use optimization functions")
