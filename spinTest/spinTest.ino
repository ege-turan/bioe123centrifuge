#define HALL_SENSOR_PIN 7  // Use an interrupt-capable pin on Arduino Micro

volatile int spinCount = 0;

void hallEffectISR() {
    spinCount++;  // Increment spin count
    Serial.println(spinCount);
}

void setup() {
    Serial.begin(9600);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);  // Internal pull-up for stability

    // Attach interrupt on falling edge (magnet passing)
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallEffectISR, FALLING);
}

void loop() {
    // Main loop does nothing, interrupt handles spin detection
}
