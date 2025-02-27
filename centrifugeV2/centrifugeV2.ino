/*
 * Centrifuge Control System
 * Centrifuge Control System with RPM Smoothing
 * 
 * Author: Ege Turan  
 * Stanford University, Winter 2025  
 * BIOE123 Bioengineering Systems Laboratory  
 * 
 * Description:
 * This Arduino program controls a centrifuge system with two states: PAUSED and ACTIVE.
 * - In the PAUSED state, the user can set the desired RPM and duration using two potentiometers.
 * - In the ACTIVE state, the system uses a PID controller to reach and maintain the target RPM.
 * - A Hall effect sensor measures the current RPM by detecting pulses from two magnets on the rotor.
 * - The system runs for the set duration before stopping automatically.
 * - A button toggles between PAUSED and ACTIVE states, with a debounce mechanism to prevent false triggers.
 * - The system status, set RPM, and either the set or remaining time are displayed on a 16x2 I2C LCD.
 * - Uses an **Exponential Moving Average (EMA)** to smooth RPM readings.
 * - PID controller maintains motor speed based on the smoothed RPM.
 * - Hall effect sensor measures RPM from two magnets on the rotor.
 * - Push button toggles system state, and an I2C LCD displays RPM & time.
 * 
 * Hardware:
 * - LCD Display (I2C, 16x2)
@@ -22,8 +20,8 @@
 * - Push button for state toggling (with external pull-up)
 * 
 * Notes:
 * - Uses the PID_v2 library for precise motor speed control.
 * - Implements a non-blocking loop for real-time updates.
 * - Uses **PID_v2** for precise control.
 * - Implements **non-blocking loop** for real-time updates.
 */

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

// Running average buffer for RPM
#define RPM_BUFFER_SIZE 10
double rpmBuffer[RPM_BUFFER_SIZE] = {0};  
int rpmIndex = 0;
double rpmSum = 0; 

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
        lastDebounceTime = millis(); 
        
        if (systemState == ACTIVE) {
            startTime = millis();
            playStartTone();
        } else {
            playStopTone();
        }
    }
    lastButtonState = reading;

    if (systemState == PAUSED) {
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
            playStopTone();
        }
    }

    // LCD update
    lcd.setCursor(0, 0);
    lcd.print(systemState == ACTIVE ? "ACTIVE " : "PAUSED ");

    lcd.setCursor(0, 1);
    lcd.print("RPM: " + String(systemState == ACTIVE ? smoothedRPM : setRPM) + "   ");

    lcd.setCursor(0, 2);
    lcd.print(systemState == ACTIVE ? ("Time Left: " + String((duration - (millis() - startTime)) / 1000) + "s  ")
                                    : ("Set Time: " + String(duration / 1000) + "s  "));
}

// **Real-time RPM Calculation with Running Average**
double readRPM() {
    unsigned long currentTime = millis();

    if (currentTime - lastRPMUpdate >= 30) { // Update every 30ms
        double elapsedTime = (currentTime - lastRPMUpdate) / 1000.0;  
        currentRPM = (totalPulseCount / elapsedTime) * 60.0;

        // Update running average buffer
        rpmSum -= rpmBuffer[rpmIndex];  // Remove oldest value
        rpmBuffer[rpmIndex] = currentRPM;  // Store new value
        rpmSum += currentRPM;  // Add new value
        rpmIndex = (rpmIndex + 1) % RPM_BUFFER_SIZE;  // Move index in buffer

        // Compute average RPM
        double avgRPM = rpmSum / RPM_BUFFER_SIZE;

        // Reset pulse count and timer
        totalPulseCount = 0;
        lastRPMUpdate = currentTime;

        // Print to Serial Monitor and Serial Plotter
        Serial.print("Raw RPM: ");
        Serial.print(currentRPM);
        Serial.print("\t Averaged RPM: ");
        Serial.println(avgRPM);
    }

    return currentRPM;
}

// **Buzzer Sounds**
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
