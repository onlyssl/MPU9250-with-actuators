#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MPU9250
#define MPU_ADDR 0x68

// Pulse sensor
#define PULSE_PIN A0
int bpm = 0;
int pulseThreshold = 550;
unsigned long lastBeatTime = 0;

// Vibration motors
#define MOTOR1 2
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 5
#define MOTOR5 6
int motors[] = {MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR5};

// Tremor detection (Z-axis only)
#define SAMPLE_RATE 50       // 50Hz
#define BUFFER_SIZE 30       // 0.6 sec buffer
int16_t gzBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool tremorDetected = false;
float tremorThreshold = 7000; // tune experimentally

// High-pass filter
float alpha = 0.96;
float gz_f = 0;
float gz_prev = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // MPU9250 Init
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Vibration motors
  for (int i = 0; i < 5; i++) pinMode(motors[i], OUTPUT);

  // Initialize buffer
  for (int i = 0; i < BUFFER_SIZE; i++) gzBuffer[i] = 0;
}

void loop() {
  readMPU();
  detectTremor();
  readPulse();
  updateOLED();
  delay(1000 / SAMPLE_RATE); // 50Hz sampling
}

// ----------------- Read MPU -----------------
int16_t gz;
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // GZ high byte register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  gz = Wire.read() << 8 | Wire.read();
}

// ----------------- Tremor Detection (RMS on Z-axis) -----------------
void detectTremor() {
  // High-pass filter
  gz_f = alpha * (gz_f + gz - gz_prev);
  gz_prev = gz;

  // Add to circular buffer
  gzBuffer[bufferIndex] = (int16_t)gz_f;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  // Compute RMS over buffer
  unsigned long sumSq = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sumSq += (long)gzBuffer[i] * gzBuffer[i];
  }
  float rms = sqrt((float)sumSq / BUFFER_SIZE);

  // Tremor detection
  if (rms > tremorThreshold) {
    tremorDetected = true;
    for (int i = 0; i < 5; i++) digitalWrite(motors[i], HIGH);
  } else {
    tremorDetected = false;
    for (int i = 0; i < 5; i++) digitalWrite(motors[i], LOW);
  }

  // Debug
  Serial.print("RMS Z: "); Serial.println(rms);
}

// ----------------- Pulse -----------------
void readPulse() {
  int pulseValue = analogRead(PULSE_PIN);
  unsigned long currentTime = millis();
  if (pulseValue > pulseThreshold && currentTime - lastBeatTime > 300) {
    bpm = 60000 / (currentTime - lastBeatTime);
    lastBeatTime = currentTime;
  }
}

// ----------------- OLED -----------------
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Parkinson's Monitor");

  display.setCursor(0, 15);
  display.print("Heart BPM: "); display.println(bpm);

  display.setCursor(0, 30);
  display.print("Tremor: ");
  if (tremorDetected) display.println("DETECTED"); else display.println("Stable");

  display.setCursor(0, 45);
  display.print("Tremor Mag: ");
  int barLength = map(rmsValue(), 0, 500, 0, 100); // tune mapping
  for (int i = 0; i < barLength; i += 4) display.print("|");

  display.display();
}

float rmsValue() {
  unsigned long sumSq = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sumSq += (long)gzBuffer[i] * gzBuffer[i];
  }
  return sqrt((float)sumSq / BUFFER_SIZE);
}
