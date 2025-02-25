#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v2.h>

// LCD setup
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Pin definitions
#define BUTTON_PIN 12
#define MOTOR_PWM 6
#define RPM_POT A0
#define TIME_POT A1
#define HALL_SENSOR_PIN 7  // Hall effect sensor pin
#define BUZZER_PIN 8

// RPM range
#define RPM_MIN 100
#define RPM_MAX 3500

// Time range
#define T_MIN 5000    // 5 sec
#define T_MAX 1800000 // 30 min

// System states
enum State { PAUSED, ACTIVE };
volatile State systemState = PAUSED;

// PID variables
double setRPM, currentRPM, outputPWM;
double smoothedRPM = 0;  // Smoothed RPM value
const double alpha = 0.2; // Smoothing factor (higher = less smoothing)
double Kp = 0.8, Ki = 0.5, Kd = 0.5;
PID motorPID(&smoothedRPM, &outputPWM, &setRPM, Kp, Ki, Kd, DIRECT);

// Timing and control variables
unsigned long startTime, duration;
volatile unsigned long totalPulseCount = 0;
unsigned long lastRPMUpdate = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms debounce delay

bool lastButtonState = HIGH;

// Interrupt handler for counting pulses
void countPulse() {
    totalPulseCount++;
}

void setup() {
    pinMode(BUTTON_PIN, INPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(HALL_SENSOR_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), countPulse, RISING);
    
    lcd.init();
    lcd.backlight();
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(0, 255);

    Serial.begin(9600);
}

void loop() {
    // Read button state with debounce
    bool reading = digitalRead(BUTTON_PIN);
    if ((millis() - lastDebounceTime) > debounceDelay && reading == LOW && lastButtonState == HIGH) {
        systemState = (systemState == PAUSED) ? ACTIVE : PAUSED;
        lastDebounceTime = millis(); // Update debounce timer
        
        if (systemState == ACTIVE) {
            startTime = millis();
            playStartTone();  // Play buzzer sound when starting
        } else {
            playStopTone();   // Play buzzer sound when stopping
        }
    }
    lastButtonState = reading;

    if (systemState == PAUSED) {
        // Read user settings
        setRPM = map(analogRead(RPM_POT), 0, 1023, RPM_MIN, RPM_MAX);
        duration = map(analogRead(TIME_POT), 0, 1023, T_MIN, T_MAX);
        analogWrite(MOTOR_PWM, 0);
    } else {
        // Read and smooth RPM
        currentRPM = readRPM();
        smoothedRPM = (alpha * currentRPM) + ((1 - alpha) * smoothedRPM);

        // Run PID control
        motorPID.Compute();
        analogWrite(MOTOR_PWM, (int)outputPWM);

        // Stop after set duration
        if ((millis() - startTime) >= duration) {
            systemState = PAUSED;
            playStopTone();  // Play buzzer sound when stopping due to timeout
        }
    }

    // LCD update (Fixed indexing to avoid overflow)
    lcd.setCursor(0, 0);
    lcd.print(systemState == ACTIVE ? "ACTIVE " : "PAUSED ");

    lcd.setCursor(0, 1);
    lcd.print("RPM: " + String(systemState == ACTIVE ? smoothedRPM : setRPM) + "   ");

    lcd.setCursor(0, 2);
    lcd.print(systemState == ACTIVE ? ("Time Left: " + String((duration - (millis() - startTime)) / 1000) + "s  ")
                                    : ("Set Time: " + String(duration / 1000) + "s  "));
}

// **Real-time RPM Calculation with Smoothing**
double readRPM() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastRPMUpdate >= 100) { // Check every 100ms
        double elapsedTime = (currentTime - lastRPMUpdate) / 1000.0;  // Convert ms to sec
        currentRPM = (totalPulseCount / elapsedTime) * 30.0;  // Adjusted for 2 magnets

        // Reset pulse count and timer
        totalPulseCount = 0;
        lastRPMUpdate = currentTime;
    }

    Serial.println(currentRPM);
    return currentRPM;
}

// **Buzzer Sounds**
void playStartTone() {
    tone(BUZZER_PIN, 1000, 200); // 1kHz for 200ms
    delay(250);
    tone(BUZZER_PIN, 1500, 200); // 1.5kHz for 200ms
}

void playStopTone() {
    tone(BUZZER_PIN, 500, 200); // 500Hz for 200ms
    delay(250);
    tone(BUZZER_PIN, 300, 200); // 300Hz for 200ms
}
