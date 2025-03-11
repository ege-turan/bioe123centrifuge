#include <SPI.h>
#include <U8g2lib.h>
#include <PID_v2.h>

// OLED setup for SPI
#define OLED_CS 10   // Chip Select
#define OLED_DC 9    // Data/Command
#define OLED_RST 8   // Reset

U8X8_SSD1309_128X64_NONAME2_4W_HW_SPI u8x8(OLED_CS, OLED_DC, OLED_RST);

// Pin definitions
#define BUTTON_PIN 12
#define MOTOR_PWM 6
#define RPM_POT A0
#define TIME_POT A1
#define HALL_SENSOR_PIN 7  // Hall effect sensor pin
#define BUZZER_PIN 11
#define LIMIT_SWITCH_PIN 5

// RPM range
#define RPM_MIN 100
#define RPM_MAX 3800 // Max stable recorded is 3720

// Time range
#define T_MIN 5000    // 5 sec
#define T_MAX 600000  // 10 min

// System states
enum State { PAUSED, ACTIVE };
volatile State systemState = PAUSED;

// PID variables
double setRPM, currentRPM, outputPWM;
double smoothedRPM = 0;  // Smoothed RPM value
const double alpha = 0.2; // Smoothing factor
double Kp = 0.7, Ki = 1.5, Kd = 0.03;
PID motorPID(&smoothedRPM, &outputPWM, &setRPM, Kp, Ki, Kd, DIRECT);

// Timing and control variables
unsigned long startTime, duration;
volatile unsigned long totalPulseCount = 0;
unsigned long lastRPMUpdate = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms debounce delay

bool lastButtonState = HIGH;

void countPulse() {
    totalPulseCount++;
}

void setup() {
    pinMode(BUTTON_PIN, INPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(HALL_SENSOR_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), countPulse, RISING);
    
    Serial.begin(9600);
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0, 0, "Centrifuge Ready");
    delay(100);
    u8x8.clear();

    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(0, 255);
}

void loop() {
    // Handle button press to toggle state
    bool reading = digitalRead(BUTTON_PIN);
    if ((millis() - lastDebounceTime) > debounceDelay && reading == LOW && lastButtonState == HIGH) {
        systemState = (systemState == PAUSED) ? ACTIVE : PAUSED;
        lastDebounceTime = millis();
        
        if (systemState == ACTIVE) {
            startTime = millis();
            playStartTone();
        } else {
            playStopTone();
        }
    }
    lastButtonState = reading;

    // Handle limit switch — Pause if triggered during ACTIVE state
    if (systemState == ACTIVE && digitalRead(LIMIT_SWITCH_PIN) == LOW) {
        systemState = PAUSED;
        playStopTone();
    }

    if (systemState == PAUSED) {
        // Read and map target RPM and time
        setRPM = map(analogRead(RPM_POT), 0, 1023, RPM_MIN, RPM_MAX);
        duration = map(analogRead(TIME_POT), 0, 1023, T_MIN, T_MAX);
        analogWrite(MOTOR_PWM, 0);
    } else {
        // Read current RPM and apply PID control
        currentRPM = readRPM();
        smoothedRPM = (alpha * currentRPM) + ((1 - alpha) * smoothedRPM);

        motorPID.Compute();
        analogWrite(MOTOR_PWM, (int)outputPWM);

        // Stop after the set time
        if ((millis() - startTime) >= duration) {
            systemState = PAUSED;
            playStopTone();
        }
    }

    updateDisplay();
}

void updateDisplay() {
    u8x8.drawString(0, 0, systemState == ACTIVE ? "ACTIVE" : "PAUSED");
    u8x8.drawString(0, 2, ("RPM: " + String(systemState == ACTIVE ? smoothedRPM : setRPM)).c_str());
    u8x8.drawString(0, 4, systemState == ACTIVE ? ("Time Left: " + String((duration - (millis() - startTime)) / 1000) + "s").c_str() 
                                              : ("Set Time: " + String(duration / 1000) + "s  ").c_str());
}

double readRPM() {
    unsigned long currentTime = millis();
    if (currentTime - lastRPMUpdate >= 30) {
        double elapsedTime = (currentTime - lastRPMUpdate) / 1000.0;
        currentRPM = (totalPulseCount / elapsedTime) * 60.0;
        totalPulseCount = 0;
        lastRPMUpdate = currentTime;
        Serial.print("RPM: ");
        Serial.println(currentRPM);
    }
    return currentRPM;
}

void playStartTone() {
    tone(BUZZER_PIN, 1000, 200); 
    delay(250);
    tone(BUZZER_PIN, 1500, 200);
}

void playStopTone() {
    tone(BUZZER_PIN, 500, 200); 
    delay(250);
    tone(BUZZER_PIN, 300, 200);
}
