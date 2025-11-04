#!/bin/bash

echo "🔧 Setting up system for Arduino + Python Display Controller..."

# Step 1: Install Homebrew if missing
if ! command -v brew &>/dev/null; then
    echo "🍺 Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

    echo "🔧 Adding Homebrew to PATH..."
    echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
    eval "$(/opt/homebrew/bin/brew shellenv)"
else
    echo "✅ Homebrew already installed."
    eval "$(/opt/homebrew/bin/brew shellenv)"
fi

# Step 2: Install Arduino IDE (modern version)
echo "🔌 Installing Arduino IDE (2.x)..."
if brew install --cask arduino-ide; then
    echo "✅ Arduino IDE installed successfully."
else
    echo "⚠️ Homebrew Arduino IDE installation failed."
    echo "➡️ Please install manually from: https://www.arduino.cc/en/software"
fi

# Step 3: Ensure Python3 is available
if command -v python3 &>/dev/null; then
    echo "✅ Python3 already installed."
else
    echo "🐍 Installing Python3..."
    brew install python
fi

# Step 4: Install Python dependencies
echo "📦 Installing Python dependencies..."
python3 -m pip install --upgrade pip
python3 -m pip install pyserial opencv-python numpy

# Step 5: Create Arduino + Python project structure
echo "💾 Creating project files..."
mkdir -p ~/Arduino_Display_Project
cd ~/Arduino_Display_Project || exit

########################################
# Arduino sketch
########################################
cat <<'EOF' > arduino_display.ino
const int numSensors = 1;
const int sensorPins[numSensors] = {A0};

// Output pin to trigger external system (black image)
const int triggerPin = 8;

void setup() {
  Serial.begin(9600);
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);  // image visible at start
}

void loop() {
  bool objectDetected = false;

  for (int i = 0; i < numSensors; i++) {
    int rawValue = analogRead(sensorPins[i]);
    float distance = rawToDistance(rawValue);

    // Debugging: print values
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
    digitalWrite(triggerPin, HIGH);  // tell system to show black image
    Serial.println("Object detected → Black image ON");
  } else {
    digitalWrite(triggerPin, LOW);   // normal image
    Serial.println("Clear → Normal image");
  }

  delay(100); // small delay for stability
}

// Convert raw analog reading to distance in cm
float rawToDistance(int rawValue) {
  // Approximation formula for GP2Y0A02YK0F
  // Distance (cm) ≈ 10650 / (rawValue - 17)
  if (rawValue <= 17) return 150; // avoid divide-by-zero
  float distance = 10650.0 / (rawValue - 17);
  if (distance > 150) distance = 150;  // cap to sensor max
  if (distance < 20) distance = 20;    // cap to sensor min
  return distance;
}
EOF

########################################
# Python script
########################################
cat <<'EOF' > display_control.py
import serial
import cv2
import numpy as np
import time

# 1️⃣ Change this to your actual Arduino port
# Windows example: 'COM3' or 'COM4'
# Linux/Mac example: '/dev/ttyACM0' or '/dev/ttyUSB0'
arduino = serial.Serial('/dev/cu.usbserial-210', 9600, timeout=1)
time.sleep(2)  # Give Arduino time to start

# 2️⃣ Load your display image
image_path = '/Users/rahuljuneja/Downloads/123.png'   # your main image file
image = cv2.imread(image_path)

if image is None:
    print("⚠️ Could not find image. Please check the path.")
    exit()

# 3️⃣ Create a black image (same size as your main image)
black_image = np.zeros_like(image)

# 4️⃣ Setup window
window_name = "Display"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

show_black = False

print("✅ Running... Press ESC to quit.")

# 5️⃣ Loop forever
while True:
    line = arduino.readline().decode(errors='ignore').strip()

    if line:
        print(line)
        if "Object detected" in line:
            show_black = True
        elif "Clear" in line:
            show_black = False

    # 6️⃣ Show appropriate image
    if show_black:
        cv2.imshow(window_name, black_image)
    else:
        cv2.imshow(window_name, image)

    # Exit on ESC key
    if cv2.waitKey(10) == 27:
        break

arduino.close()
cv2.destroyAllWindows()
EOF

########################################
# Wrap-up
########################################
echo ""
echo "🎉 Setup complete!"
echo "➡️ Open Arduino IDE, load ~/Arduino_Display_Project/arduino_display.ino, and upload to your Arduino."
echo "➡️ Then run this command in Terminal after plugging in your Arduino:"
echo ""
echo "   python3 ~/Arduino_Display_Project/display_control.py"
echo ""
echo "💡 Tip: Update the correct serial port path in display_control.py (e.g., '/dev/cu.usbserial-XXX')."
