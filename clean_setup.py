#!/bin/bash

echo "🧹 Starting cleanup of Arduino + Python Display setup..."

########################################
# 1. Remove project files
########################################
if [ -d "$HOME/Arduino_Display_Project" ]; then
    echo "🗑 Removing ~/Arduino_Display_Project..."
    rm -rf "$HOME/Arduino_Display_Project"
else
    echo "✅ No project folder found."
fi

########################################
# 2. Uninstall Python venv dependencies
########################################
if [ -d "$HOME/Arduino_Display_Project/venv" ]; then
    echo "🧯 Removing Python virtual environment..."
    rm -rf "$HOME/Arduino_Display_Project/venv"
else
    echo "✅ No virtual environment found."
fi

########################################
# 3. Uninstall Arduino IDE (if installed via Homebrew)
########################################
if brew list --cask | grep -q "arduino-ide"; then
    echo "🧩 Uninstalling Arduino IDE..."
    brew uninstall --cask arduino-ide
else
    echo "✅ Arduino IDE not found via Homebrew."
fi

########################################
# 4. Optionally remove Homebrew Python
########################################
if brew list | grep -q "^python@"; then
    echo "🐍 Removing Homebrew Python..."
    brew uninstall python || brew uninstall python@3
else
    echo "✅ No Homebrew Python installation found."
fi

########################################
# 5. Clean up Homebrew cache
########################################
echo "🧽 Cleaning Homebrew cache..."
brew cleanup -s
rm -rf "$(brew --cache)"

########################################
# 6. (Optional) Uninstall Homebrew itself
########################################
# ⚠️ Uncomment the next line if you truly want to remove Homebrew entirely
# /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/uninstall.sh)"

echo ""
echo "✅ Cleanup complete!"
echo "💡 Note: Homebrew and system Python may still be installed unless you chose to remove them manually."
