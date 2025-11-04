#!/bin/bash

echo "🔧 Setting up system for Arduino + Python Display Controller (sandboxed)..."

########################################
# 0. Pre-flight check
########################################
set -e  # Stop on errors
trap 'echo "❌ Setup failed at line $LINENO. Check the output above."' ERR

# Ask for sudo at the start
sudo -v

########################################
# 1. Install / Repair Homebrew
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

# Fix Homebrew permissions
echo "🛠 Fixing Homebrew permissions..."
sudo chown -R "$(whoami)" /opt/homebrew
sudo chmod -R u+w /opt/homebrew
rm -f /opt/homebrew/var/homebrew/locks/*.lock 2>/dev/null || true

########################################
# 2. Install Arduino IDE (modern version)
########################################
echo "🔌 Installing Arduino IDE (2.x)..."
if brew install --cask arduino-ide; then
    echo "✅ Arduino IDE installed successfully."
else
    echo "⚠️ Homebrew Arduino IDE installation failed."
    echo "➡️ Please install manually from: https://www.arduino.cc/en/software"
fi

########################################
# 3. Ensure Python3 is available
########################################
if ! command -v python3 &>/dev/null; then
    echo "🐍 Installing Python3..."
    brew install python
else
    echo "✅ Python3 already installed."
fi

########################################
# 4. Create project folder
########################################
PROJECT_DIR="$HOME/Arduino_Display_Project"
echo "📂 Setting up project directory at: $PROJECT_DIR"
mkdir -p "$PROJECT_DIR"
cd "$PROJECT_DIR" || exit

########################################
# 5. Create virtual environment
########################################
if [ -d "venv" ]; then
    echo "♻️ Existing virtual environment found. Reusing..."
else
    echo "📦 Creating Python virtual environment..."
    python3 -m venv venv
fi

source venv/bin/activate

########################################
# 6. Install Python dependencies
########################################
echo "📦 Installing Python dependencies..."
pip install --upgrade pip
pip install pyserial opencv-python numpy

########################################
# 7. Arduino sketch
########################################
echo "🧩 Writing Arduino sketch..."
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
# 8. Python display control script
########################################
echo "🖥 Creating Python display control script..."
cat <<'EOF' > display_control.py
import serial
import cv2
import numpy as np
import time
import sys

SERIAL_PORT = '/dev/cu.usbserial-210'  # Update if needed
BAUD_RATE = 9600
IMAGE_PATH = '/Users/rahuljuneja/Downloads/123.png'

print("🔌 Connecting to Arduino...")
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print(f"⚠️ Could not open port {SERIAL_PORT}. Please update the path.")
    sys.exit(1)

image = cv2.imread(IMAGE_PATH)
if image is None:
    print(f"⚠️ Could not find image at {IMAGE_PATH}")
    sys.exit(1)

black_image = np.zeros_like(image)
window_name = "Display"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

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

    frame = black_image if show_black else image
    cv2.imshow(window_name, frame)

    if cv2.waitKey(10) == 27:
        break

arduino.close()
cv2.destroyAllWindows()
EOF

########################################
# 9. Run helper
########################################
cat <<'EOF' > run.sh
#!/bin/bash
cd ~/Arduino_Display_Project || exit
source venv/bin/activate
python3 display_control.py
EOF
chmod +x run.sh

########################################
# 10. Optional Self-Test
########################################
if [[ "$1" == "--self-test" ]]; then
    echo "🧪 Running self-test (no Arduino needed)..."
    source venv/bin/activate
    python3 - <<'PYCODE'
import cv2, numpy as np, time
print("✅ OpenCV test window will appear for 2 seconds...")
img = np.zeros((300, 300, 3), dtype=np.uint8)
cv2.putText(img, "Self-Test OK", (40, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
cv2.imshow("Test", img)
cv2.waitKey(2000)
cv2.destroyAllWindows()
PYCODE
fi

########################################
# 11. Wrap-up
########################################
echo ""
echo "🎉 Setup complete!"
echo "➡️ Open Arduino IDE and upload:"
echo "   $PROJECT_DIR/arduino_display.ino"
echo ""
echo "➡️ Then run the display system with:"
echo "   bash $PROJECT_DIR/run.sh"
echo ""
echo "💡 Tip: Use '--self-test' when running setup to verify OpenCV display works."
