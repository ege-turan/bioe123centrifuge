/*
 * Centrifuge V1c
 * 
 * Author: Ege Turan  
 * Stanford University, Winter 2025  
 * BIOE123 Bioengineering Systems Laboratory  
 *
 * Simple PWM NMOS DC motor driver for a centrifuge prototype
 */

// Pin definitions
const int POT_PIN = A0; // Potentiometer connected to A0
const int NMOS_GATE_PIN = 6; // NMOS gate connected to D6 (PWM pin)

// Setup function
void setup() {
  pinMode(NMOS_GATE_PIN, OUTPUT); // Set D6 as an output
}

// Main loop
void loop() {
  // Read the potentiometer value (0 to 1023)
  int potValue = analogRead(POT_PIN);

  // Map the potentiometer value to PWM range (0 to 255)
  int pwmValue = map(potValue, 0, 1023, 0, 255);

  // Write the PWM value to the NMOS gate pin
  analogWrite(NMOS_GATE_PIN, pwmValue);

  // Optional: Small delay for stability
  delay(10);
}
