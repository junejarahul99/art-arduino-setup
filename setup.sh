#!/bin/bash
echo "🔧 Setting up system for Arduino + Python display control..."

# Step 1: Install Homebrew if not already installed
if ! command -v brew &> /dev/null
then
    echo "🍺 Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

    # Add Homebrew to PATH immediately for this session
    if [ -d "/opt/homebrew/bin" ]; then
        echo "🔗 Adding Homebrew to PATH for current session..."
        eval "$(/opt/homebrew/bin/brew shellenv)"
    fi
else
    echo "✅ Homebrew already installed."
    eval "$(/opt/homebrew/bin/brew shellenv)"
fi

# Step 1b: Confirm Homebrew is usable
if ! command -v brew &> /dev/null; then
    echo "❌ Homebrew not found even after installation."
    echo "➡️ Please restart Terminal and re-run this command."
    exit 1
fi

# Step 2: Install Arduino IDE
if ! brew list --cask | grep -q "arduino"; then
    echo "🔌 Installing Arduino IDE..."
    brew install --cask arduino
else
    echo "✅ Arduino IDE already installed."
fi

# Step 3: Install Python3
if ! command -v python3 &> /dev/null
then
    echo "🐍 Installing Python3..."
    brew install python
else
    echo "✅ Python3 already installed."
fi

# Step 4: Install Python dependencies
echo "📦 Installing Python dependencies..."
python3 -m pip install --upgrade pip --no-warn-script-location
python3 -m pip install pyserial opencv-python numpy --no-warn-script-location

# Step 5: Create a folder for the project
mkdir -p ~/Arduino_Display_Project
cd ~/Arduino_Display_Project || exit

# Step 6: Create project files
echo "💾 Creating project files..."

# Arduino sketch
cat << 'EOF' > arduino_display.ino
// Arduino distance-based trigger
const int numSensors = 1;
const int sensorPins[numSensors] = {A0};
const int triggerPin = 8;

void setup() {
  Serial.begin(9600);
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);  // Image visible at start
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
  if (rawValue <= 17) return 150; // Avoid divide-by-zero
  float distance = 10650.0 / (rawValue - 17);
  if (distance > 150) distance = 150;
  if (distance < 20) distance = 20;
  return distance;
}
EOF

# Python script
cat << 'EOF' > display_control.py
import serial
import cv2
import numpy as np
import time
import sys
import glob
import os

def find_arduino_port():
    """Auto-detect Arduino serial port"""
    ports = glob.glob('/dev/cu.usb*') + glob.glob('/dev/ttyUSB*')
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            return port
        except (OSError, serial.SerialException):
            pass
    return None

print("🔍 Searching for Arduino...")
port = find_arduino_port()
if not port:
    print("⚠️ No Arduino detected. Please plug it in and try again.")
    sys.exit(1)
print(f"✅ Arduino detected on {port}")

arduino = serial.Serial(port, 9600, timeout=1)
time.sleep(2)

image_path = '/Users/rahuljuneja/Downloads/123.png'
image = cv2.imread(image_path)
if image is None:
    print("⚠️ Could not find image file at", image_path)
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

    if show_black:
        cv2.imshow(window_name, black_image)
    else:
        cv2.imshow(window_name, image)

    if cv2.waitKey(10) == 27:  # ESC
        break

arduino.close()
cv2.destroyAllWindows()
EOF

echo "🎉 Setup complete!"
echo
echo "➡️ Open Arduino IDE, load ~/Arduino_Display_Project/arduino_display.ino, and upload to your Arduino."
echo "➡️ Then run this command in Terminal after plugging in Arduino:"
echo "   python3 ~/Arduino_Display_Project/display_control.py"
echo
echo "💡 Tip: To re-run anytime, simply execute the same command again. It’s safe and repeatable."
