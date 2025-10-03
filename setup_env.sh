#!/usr/bin/env bash
# Setup Python virtual environment and install dependencies

set -e

# Create venv if not exists
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi

# Activate environment
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install dependencies
pip install -r requirements.txt

echo "✅ Environment setup complete."
echo "To activate it later, run:"
echo "source venv/bin/activate"
