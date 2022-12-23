#include <Arduino.h>

// The resolution of the analog read.
// This is specific to the Arduino board!
#define ADC_RESOLUTION_BITS 12

// ============================================================================

struct Vec3i {
  uint32_t x;
  uint32_t y;
  uint32_t z;
};

struct Vec3f {
  float x;
  float y;
  float z;
};

// ============================================================================

// Convert radians to degrees
float radiansToDegrees(float value) {
  return value * (180.0/3.141592653589793238463);
}

// Get an angle (in radians) between vector A and B
float vectorAngle(const Vec3f& a, const Vec3f& b) {
  const auto dot = a.x * b.x + a.y * b.y + a.z * b.z;
  const auto sq1 = a.x * a.x + a.y * a.y + a.z * a.z;
  const auto sq2 = b.x * b.x + b.y * b.y + b.z * b.z;
  return acos(dot / sqrt(sq1 * sq2));
}

// Returns the length of the vector
float vectorLength(const Vec3f& vec) {
  return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

// Normalize the vector (length of the vector will be exactly 1.0)
Vec3f vectorNormalize(const Vec3f& vec) {
  const float length = vectorLength(vec);
  Vec3f res;
  res.x = vec.x / length;
  res.y = vec.y / length;
  res.z = vec.z / length;
  return res;
}

// ============================================================================

class MMA7361 {
public:
  explicit MMA7361(uint32_t pinX, uint32_t pinY, uint32_t pinZ) 
    : pinX{pinX}, 
      pinY{pinY},
      pinZ{pinZ} {

    pinMode(pinX, INPUT);
    pinMode(pinY, INPUT);
    pinMode(pinZ, INPUT);
  }

  Vec3i readRaw() {
    Vec3i vec;
    vec.x = analogRead(pinX);
    vec.y = analogRead(pinY);
    vec.z = analogRead(pinZ);
    return vec;
  }

  Vec3f read() {
    Vec3i raw = readRaw();
    Vec3f vec;
    vec.x = map(raw.x, min.x, max.x, 0, 1000) / 500.0f - 1.0f;
    vec.y = map(raw.y, min.y, max.y, 0, 1000) / 500.0f - 1.0f;
    vec.z = map(raw.z, min.z, max.z, 0, 1000) / 500.0f - 1.0f;
    return vectorNormalize(vec);
  }

  void setRangeMin(Vec3i value) {
    min = value;
  }

  void setRangeMax(Vec3i value) {
    max = value;
  }

private:
  uint32_t pinX;
  uint32_t pinY;
  uint32_t pinZ;

  Vec3i min;
  Vec3i max;
};

// ============================================================================

// The accelerometer instance.
// Reads the XYZ values via the analog ports
MMA7361 accelerometer(A0, A1, A2);

// Which way is the up?
// Relative to the accelerometer module!
Vec3f up{0.0f, 0.0f, 1.0f};

// ============================================================================

void setup() {
  // Serial port setup
  Serial.begin(115200);

  // Set analogRead resolution
  analogReadResolution(ADC_RESOLUTION_BITS);

  // Accelerometer calibration values
  // Minimum values for the X, Y, and Z axis
  accelerometer.setRangeMin(Vec3i{1040, 1340, 800});
  // Maximum values for the X, Y, and Z axis
  accelerometer.setRangeMax(Vec3i{3040, 3300, 2700});

  // Button
  pinMode(D5, INPUT_PULLUP);
}

// ============================================================================

void loop() {
  // Code for calibration
  /*
  Vec3i raw = accelerometer.readRaw();
  Serial.print("Raw is: [");
  Serial.print(raw.x);
  Serial.print(", ");
  Serial.print(raw.y);
  Serial.print(", ");
  Serial.print(raw.z);
  Serial.println("]");
  */
  
  // Get the unit vector (float) of acceleration
  Vec3f vec = accelerometer.read();

  // Press the button to reset the reference point.
  if (digitalRead(D5) == LOW) {
    Serial.println("Reset");
    up = vec;
  }

  // Get the angle between the vector of the acceleration and the "up" vector.
  const float angle = radiansToDegrees(vectorAngle(vec, up));

  Serial.print("Value is: [");
  Serial.print(vec.x, 4);
  Serial.print(", ");
  Serial.print(vec.y, 4);
  Serial.print(", ");
  Serial.print(vec.z, 4);
  Serial.print("] angle: ");
  Serial.print(angle, 1);
  Serial.println("");

  delay(100);
}
