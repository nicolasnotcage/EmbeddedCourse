#include <Arduino.h>

// Pin Definitions
#define PD_PIN 2         // IR receiver power control pin
#define LED 12           // Status LED pin
#define sensorRead A0    // IR receiver output pin
#define ENABLE 5         // Motor enable pin (PWM control)
#define DIRA 4           // Motor direction pin A
#define DIRB 6           // Motor direction pin B
#define BUTTON_PIN 3     // Button pin for start/stop control

// Variables
volatile unsigned int rotations = 0;      // Counter for blade passes
volatile bool bladeDetected = false;      // Flag for blade detection
volatile bool calculateFlag = false;      // Flag for RPM calculation
bool systemRunning = false;               // Flag to check if the system is running

const unsigned long rpmInterval = 1000;   // Interval for RPM calculation (1 second)
const unsigned long numBlades = 3;        // Number of blades on the fan
const unsigned long oneMinute = 60000;    // One minute in milliseconds
const int limit = 850;                    // IR detection threshold
unsigned long lastRPMTime = 0;            // Time of last RPM calculation

volatile unsigned long lastDebounceTime = 0; // Time of the last button press
const unsigned long debounceDelay = 200;     // Minimum debounce time (in milliseconds)

// Declare functions
void calculateRPM();
void irISR();

void setup() 
{
  // LED and IR setup
  pinMode(PD_PIN, INPUT);     // IR receiver input pin
  pinMode(LED, OUTPUT);       // Status LED pin
  digitalWrite(PD_PIN, HIGH); // IR receiver powered on
  digitalWrite(LED, LOW);     // Status LED off initially

  // Motor setup
  pinMode(ENABLE, OUTPUT);    // Motor enable pin
  pinMode(DIRA, OUTPUT);      // Motor direction pin A 
  pinMode(DIRB, OUTPUT);      // Motor direction pin B
  digitalWrite(DIRA, LOW);   // Set motor direction A
  digitalWrite(DIRB, HIGH);    // Set motor direction B

  // Button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Enable internal pull-up resistor

  // Attach interrupt for the IR receiver pin (PD_PIN)
  attachInterrupt(digitalPinToInterrupt(PD_PIN), irISR, FALLING);

  // Serial setup
  Serial.begin(9600);

  // Set up an internal 1s timer interrupt. Used Arduino interrupts timer 
  // calculator tool deepbluembedded website to generate values and code.
  cli();                // Disable global interrupts
  TCCR1A = 0;           // Set TCCR1A register to 0
  TCCR1B = 0;           // Set TCCR1B register to 0
  TCCR1B |= B00000100;  // Prescaler = 256
  OCR1A = 62500;        // Timer Compare1A Register
  TIMSK1 |= B00000010;  // Enable Timer COMPA Interrupt
  sei();                // Enable global interrupts
}

void loop() 
{
  // Read the button state
  bool buttonState = digitalRead(BUTTON_PIN);

  // Check for button press (active LOW)
  if (buttonState == LOW && (millis() - lastDebounceTime) > debounceDelay) 
  {
    lastDebounceTime = millis(); // Update debounce timer

    // Toggle system running state
    systemRunning = !systemRunning;
  }

  if (systemRunning) 
  {
    // System is running, start motor at full speed and process RPM
    analogWrite(ENABLE, 255); 
    
    // Flash Status LED on Blade Detection
    if (bladeDetected) 
    {
      bladeDetected = false; // Reset flag

      // Flash the LED
      digitalWrite(LED, HIGH);
      delay(100); 
      digitalWrite(LED, LOW);
    }

    // RPM Calculation Every Second
    if (calculateFlag) 
    {
      calculateFlag = false;      // Reset flag
      calculateRPM();             // Calculate RPM
    }
  } 
  else 
  {
    analogWrite(ENABLE, 0); // Stop motor
    rotations = 0;          // Reset rotations
  }
}

// Interrupt Service Routine for Timer1
ISR(TIMER1_COMPA_vect)
{
  OCR1A += 62500;
  calculateFlag = true; // Set flag for RPM calculation
}

// ISR is called when the IR beam is interrupted
void irISR() 
{
  rotations++;          // Increment rotations
  bladeDetected = true; // Set flag for blade detection
}

// RPM Calculation Function
void calculateRPM() 
{
  cli(); // Disable global interrupts during calculation
  float rpm = (rotations / numBlades) * (oneMinute / rpmInterval); // Calculate and print RPM based on blade count
  sei(); // Enable global interrupts after calculation

  // Print time and RPM
  Serial.print(millis());
  Serial.print(",");
  Serial.println(rpm);

  rotations = 0; // Reset blade counter for the next interval
}
