#!/bin/bash

echo "🔧 Setting up system for Arduino + Python Display Controller (sandboxed)..."

set -e  # stop on any error
trap 'echo "❌ Setup failed at line $LINENO. Check the message above."' ERR

sudo -v  # request admin once upfront

########################################
# 1. Install / Fix Homebrew
########################################
if ! command -v brew &>/dev/null; then
    echo "🍺 Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
    eval "$(/opt/homebrew/bin/brew shellenv)"
else
    echo "✅ Homebrew already installed."
    eval "$(/opt/homebrew/bin/brew shellenv)"
fi

########################################
# 2. Auto-fix Homebrew permissions
########################################
echo "🛠 Checking Homebrew permissions..."
BREW_PREFIX=$(brew --prefix)

if [ "$(stat -f '%Su' $BREW_PREFIX)" != "$(whoami)" ]; then
    echo "🧩 Fixing ownership of Homebrew folders..."
    sudo chown -R "$(whoami)":staff "$BREW_PREFIX"
fi

echo "🧹 Cleaning old Homebrew locks..."
sudo rm -rf "$BREW_PREFIX/var/homebrew/locks"/*.lock 2>/dev/null || true
sudo mkdir -p "$BREW_PREFIX/var/homebrew/locks"
sudo chown -R "$(whoami)":staff "$BREW_PREFIX/var/homebrew"
chmod -R u+w "$BREW_PREFIX"

echo "🔄 Running brew doctor + cleanup..."
brew doctor || true
brew cleanup -s || true
brew update --force --quiet || true

########################################
# 3. Install Arduino IDE
########################################
echo "🔌 Installing Arduino IDE (2.x)..."
if [ -d "/Applications/Arduino IDE.app" ]; then
    echo "🧹 Removing old Arduino IDE..."
    sudo rm -rf "/Applications/Arduino IDE.app"
fi

if brew install --cask arduino-ide; then
    echo "✅ Arduino IDE installed successfully."
else
    echo "⚠️ Arduino IDE installation failed — manual install recommended:"
    echo "➡️ https://www.arduino.cc/en/software"
fi

########################################
# 4. Install Python 3 if missing
########################################
if ! command -v python3 &>/dev/null; then
    echo "🐍 Installing Python3..."
    brew install python
else
    echo "✅ Python3 already installed."
fi

########################################
# 5. Set up project directory
########################################
PROJECT_DIR="$HOME/Arduino_Display_Project"
mkdir -p "$PROJECT_DIR"
cd "$PROJECT_DIR" || exit
echo "📂 Project directory: $PROJECT_DIR"

########################################
# 6. Copy image (expects work.jpg in same folder as this script)
########################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/work.jpg" ]; then
    echo "🖼️ Copying work.jpg to project folder..."
    cp "$SCRIPT_DIR/work.jpg" "$PROJECT_DIR/work.jpg"
else
    echo "⚠️ No work.jpg found next to setup.sh. Please place it beside the script before running."
fi

########################################
# 7. Virtual environment + dependencies
########################################
if [ ! -d "venv" ]; then
    echo "📦 Creating Python virtual environment..."
    python3 -m venv venv
fi

source venv/bin/activate
echo "📦 Installing dependencies..."
pip install --upgrade pip
pip install pyserial opencv-python numpy

########################################
# 8. Arduino sketch
########################################
cat <<'EOF' > arduino_display.ino
const int numSensors = 1;
const int sensorPins[numSensors] = {A0};
const int triggerPin = 8;

void setup() {
  Serial.begin(9600);
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
}

void loop() {
  bool objectDetected = false;

  for (int i = 0; i < numSensors; i++) {
    int rawValue = analogRead(sensorPins[i]);
    float distance = rawToDistance(rawValue);

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance >= 20 && distance <= 90) {
      objectDetected = true;
    }
  }

  if (objectDetected) {
    digitalWrite(triggerPin, HIGH);
    Serial.println("Object detected → Black image ON");
  } else {
    digitalWrite(triggerPin, LOW);
    Serial.println("Clear → Normal image");
  }

  delay(100);
}

float rawToDistance(int rawValue) {
  if (rawValue <= 17) return 150;
  float distance = 10650.0 / (rawValue - 17);
  if (distance > 150) distance = 150;
  if (distance < 20) distance = 20;
  return distance;
}
EOF

########################################
# 9. Python display control
########################################
cat <<'EOF' > display_control.py
import serial
import cv2
import numpy as np
import time
import sys
import os

SERIAL_PORT = '/dev/cu.usbserial-210'  # Change if needed
BAUD_RATE = 9600
IMAGE_PATH = os.path.expanduser('~/Arduino_Display_Project/work.jpg')

print("🔌 Connecting to Arduino...")
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print(f"⚠️ Could not open {SERIAL_PORT}. Please check your Arduino port.")
    sys.exit(1)

image = cv2.imread(IMAGE_PATH)
if image is None:
    print(f"⚠️ Could not load image at {IMAGE_PATH}")
    sys.exit(1)

black_image = np.zeros_like(image)
window = "Display"
cv2.namedWindow(window, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

show_black = False
print("✅ Running... Press ESC to quit.")

while True:
    line = arduino.readline().decode(errors='ignore').strip()
    if line:
        print(line)
        if "Object detected" in line:
            show_black = True
        elif "Clear" in line:
            show_black = False

    cv2.imshow(window, black_image if show_black else image)
    if cv2.waitKey(10) == 27:
        break

arduino.close()
cv2.destroyAllWindows()
EOF

########################################
# 10. Run helper
########################################
cat <<'EOF' > run.sh
#!/bin/bash
cd ~/Arduino_Display_Project || exit
source venv/bin/activate
python3 display_control.py
EOF
chmod +x run.sh

########################################
# 11. Wrap-up
########################################
echo ""
echo "🎉 Setup complete!"
echo "➡️ Open Arduino IDE, load ~/Arduino_Display_Project/arduino_display.ino, and upload it to your Arduino."
echo "➡️ Then to run the display system, use:"
echo ""
echo "   bash ~/Arduino_Display_Project/run.sh"
echo ""
echo "💡 Make sure 'work.jpg' is beside setup.sh before you start."
