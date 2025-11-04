#!/bin/bash
echo "🔧 Setting up system for Arduino + Python display control..."

# Step 1: Install Homebrew if not already installed
if ! command -v brew &> /dev/null
then
    echo "🍺 Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
else
    echo "✅ Homebrew already installed."
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

# Step 4: Install pip packages
echo "📦 Installing Python dependencies..."
python3 -m pip install --upgrade pip
python3 -m pip install pyserial opencv-python numpy

# Step 5: Create a folder for the project
mkdir -p ~/Arduino_Display_Project
cd ~/Arduino_Display_Project

# Step 6: Create placeholder files
echo "💾 Creating project files..."

# Arduino sketch
cat << 'EOF' > arduino_display.ino
// Your Arduino code pasted here
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
    Serial.print("Sensor "); Serial.print(i);
    Serial.print(": "); Serial.print(distance); Serial.println(" cm");
    if (distance >= 20 && distance <= 90) objectDetected = true;
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

# Python script
cat << 'EOF' > display_control.py
import serial
import cv2
import numpy as np
import time

arduino = serial.Serial('/dev/cu.usbserial-210', 9600, timeout=1)
time.sleep(2)
image_path = '/Users/rahuljuneja/Downloads/123.png'
image = cv2.imread(image_path)
if image is None:
    print("⚠️ Could not find image file.")
    exit()
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
    if cv2.waitKey(10) == 27:
        break
arduino.close()
cv2.destroyAllWindows()
EOF

echo "🎉 Setup complete!"
echo "➡️ Open Arduino IDE, load ~/Arduino_Display_Project/arduino_display.ino, and upload to Arduino."
echo "➡️ Then run 'python3 ~/Arduino_Display_Project/display_control.py' in Terminal after plugging in Arduino."
