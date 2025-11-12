#!/bin/bash
# Setup script for MAVLink HAL Configuration Tools
# Creates Python virtual environment and installs dependencies

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "================================"
echo "MAVLink HAL Config Tools Setup"
echo "================================"
echo

# Check Python 3
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed"
    exit 1
fi

PYTHON_VERSION=$(python3 --version)
echo "Found: $PYTHON_VERSION"
echo

# Create venv if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv
    echo "✓ Virtual environment created"
else
    echo "✓ Virtual environment already exists"
fi

echo

# Activate venv
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --quiet --upgrade pip

# Install dependencies
echo "Installing dependencies..."
pip install --quiet -r requirements.txt

echo
echo "✓ Dependencies installed:"
pip list | grep -E "jsonschema|pyyaml|colorama|tabulate|jinja2"

echo
echo "================================"
echo "Setup Complete!"
echo "================================"
echo
echo "To use the tools:"
echo "  1. Activate environment:  source venv/bin/activate"
echo "  2. Validate schema:       python validation/config_validator.py --schema"
echo "  3. Validate examples:     python validation/config_validator.py --all"
echo
echo "To deactivate:              deactivate"
echo
