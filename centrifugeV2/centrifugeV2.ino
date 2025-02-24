/*
 * Centrifuge Control System
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
 * 
 * Hardware:
 * - LCD Display (I2C, 16x2)
 * - Hall effect sensor for RPM measurement
 * - NMOS transistor for motor control (PWM on pin 7)
 * - Two potentiometers for setting RPM and time
 * - Push button for state toggling (with external pull-up)
 * 
 * Notes:
 * - Uses the PID_v2 library for precise motor speed control.
 * - Implements a non-blocking loop for real-time updates.
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

// RPM range
#define RPM_MIN 100
#define RPM_MAX 3500

// Time range
#define T_MIN 5000    // 5 sec
#define T_MAX 1800000 // 3 min

// RPM averaging window
#define RPM_AVG_WINDOW 3  // Time window (in milliseconds)

// System states
enum State { PAUSED, ACTIVE };
volatile State systemState = PAUSED;

// PID variables
double setRPM, currentRPM, outputPWM;
double Kp = 0.8, Ki = 0.5, Kd = 0.5;  // Tuned PID constants
PID motorPID(&currentRPM, &outputPWM, &setRPM, Kp, Ki, Kd, DIRECT);

// Timing and control variables
unsigned long startTime, duration;
volatile unsigned long totalPulseCount = 0;
unsigned long timeBufferStart = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms debounce delay

bool lastButtonState = HIGH;
bool currentButtonState;

// Interrupt handler for counting pulses
void countPulse() {
    totalPulseCount++;
}

void setup() {
    pinMode(BUTTON_PIN, INPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(HALL_SENSOR_PIN, INPUT);
    
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
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading == LOW && currentButtonState == HIGH) {
            systemState = (systemState == PAUSED) ? ACTIVE : PAUSED;
            if (systemState == ACTIVE) startTime = millis();
        }
        currentButtonState = reading;
    }
    lastButtonState = reading;

    if (systemState == PAUSED) {
        // Read settings
        setRPM = map(analogRead(RPM_POT), 1023, 0, RPM_MIN, RPM_MAX); // RPM range
        duration = map(analogRead(TIME_POT), 1023, 0, T_MIN, T_MAX); // Time range
        analogWrite(MOTOR_PWM, 0);
    } else {
        // Read current RPM with a time-based averaging window
        currentRPM = readRPM();
        
        // PID control
        motorPID.Compute();
        analogWrite(MOTOR_PWM, (int)outputPWM);

        // Stop after time runs out
        unsigned long elapsedTime = millis() - startTime;
        if (elapsedTime >= duration) {
            systemState = PAUSED;
        }
    }
    
    // LCD update
    lcd.setCursor(0, 0);
    lcd.print(systemState == ACTIVE ? "ACTIVE        " : "PAUSED        ");
    lcd.setCursor(0, 1);
    lcd.print(systemState == ACTIVE ? ("cur RPM: " + String(currentRPM) + " ") : ("set RPM: " + String(setRPM) + " "));
    lcd.setCursor(0, 2);
    if (systemState == ACTIVE) {
        unsigned long remainingTime = (duration - (millis() - startTime)) / 1000;
        lcd.print("rem time: " + String(remainingTime) + "s  ");
    } else {
        lcd.print("set time: " + String(duration / 1000) + "s  ");
    }
}

// **Improved RPM Calculation with Time Buffering**
double readRPM() {
    unsigned long currentTime = millis();
    
    // Check if enough time has passed to compute RPM
    if (currentTime - timeBufferStart >= RPM_AVG_WINDOW) {
        double elapsedTime = (currentTime - timeBufferStart) / 1000.0;  // Convert ms to seconds
        currentRPM = (totalPulseCount / elapsedTime) * 60.0;  // Convert to RPM
        
        // Reset buffer
        totalPulseCount = 0;
        timeBufferStart = currentTime;
    }

    Serial.println(currentRPM);
    return currentRPM;
}
