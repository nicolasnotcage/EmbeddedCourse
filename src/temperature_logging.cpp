#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME280_I2C_ADDRESS 0x76  // Default I2C address for BME280
#define SD_CS_PIN 10             // Chip Select pin for SD card reader
#define BUTTON_PIN 2             // Interrupt button pin

Adafruit_BME280 bme;             // BME280 object
File dataFile;                   // File object for SD card

volatile bool buttonPressed = false;      // Track button state
volatile bool logTemperatureFlag = false; // Track if temperature should be 
                                          // logged

unsigned long debounceTime = 0;          // Time of last button press
const unsigned long debounceDelay = 200; // Debounce delay in milliseconds

// Declare functions
void buttonISR();
void logTemperature();
void transferDataToSerial();

void setup() 
{
  // Initialize Serial Communication
  Serial.begin(9600);

  // Initialize BME280 sensor
  if (!bme.begin(BME280_I2C_ADDRESS)) 
  {
    // Loop forever if BME280 initialization fails. Future refactors may add 
    // retry logic or flashing LEDs to indicate failure.
    while (1);
  }

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) 
  {
    // Loop forever if SD Card initialization fails. Future refactors may add 
    // retry logic or flashing LEDs to indicate failure.
    while (1);
  }

  // Check if the csv file exists; otherwise create it and write header
  if (!SD.exists("tempdata.csv")) 
  {
    dataFile = SD.open("tempdata.csv", FILE_WRITE);
    if (dataFile) 
    {
      dataFile.println("Timestamp(ms),Temperature(F)");
      dataFile.close();
    }
  }

  // Set up an internal 1s timer interrupt. Used Arduino interrupts timer 
  // calculator tool deepbluembedded website to generate values and code.
  cli();                // Disable global interrupts
  TCCR1A = 0;           // Set TCCR1A register to 0
  TCCR1B = 0;           // Set TCCR1B register to 0
  TCCR1B |= B00000100;  // Prescaler = 256
  OCR1A = 62500;        // Timer Compare1A Register
  TIMSK1 |= B00000010;  // Enable Timer COMPA Interrupt
  sei();                // Enable global interrupts
  
  // Set up button as an interrupt. We use the internal pull-up resistor to
  // ensure no false button presses detected. Interrupt triggers on a falling 
  // edge.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING); 
}

void loop() 
{
  // Check if it's time to take a temperature reading
  if (logTemperatureFlag) 
  {
    // Reset temperature flag
    logTemperatureFlag = false;
    
    // Log temperature to SD card
    logTemperature();
  }

  // Check if button has been pressed and serial connection is available
  if (buttonPressed) 
  {
    // Reset button pressed flag
    buttonPressed = false;

    // Transfer data if serial connection is available
    if (Serial)
    {
      // Transfer CSV data to serial
      transferDataToSerial();
    }
  }
}

// Interrupt Service Routine for Timer1
ISR(TIMER1_COMPA_vect)
{
  static int count = 0; // Counter for number of seconds; used to count to 10
  count++;              // Increment counter
  OCR1A += 62500;       // Advance The COMPA Register

  if (count >= 10)
  {
    count = 0;
    logTemperatureFlag = true;
  }  
}

// ISR function for button press
void buttonISR() 
{
  // Check debounce
  if ((millis() - debounceTime) > debounceDelay) 
  {
    buttonPressed = true;
    debounceTime = millis();
  }
}

// Function to read temperature and log to SD card
void logTemperature() 
{
  // Read temperature from BME280
  float temperature = bme.readTemperature();

  // Convert temperature to Fahrenheit
  temperature = (temperature * 1.8) + 32;

  // Open the file stored on the SD card for writing
  dataFile = SD.open("tempdata.csv", FILE_WRITE);
  if (dataFile) 
  {
    // Log timestamp and temperature to file
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.println(temperature);
    dataFile.close();
  } 
  else 
  {
    Serial.println("Error opening tempdata.csv for logging");
  }
}

// Function to transfer CSV data to serial
void transferDataToSerial()
{
  // Open the file stored on the SD card for reading
  dataFile = SD.open("tempdata.csv");

  if (dataFile) 
  {
    // Flush contents of data file to serial
    while (dataFile.available()) 
    {
      Serial.write(dataFile.read());
    }

    // Close the file
    dataFile.close();
  } 
  else 
  {
    Serial.println("Error opening tempdata.csv for data transfer");
  }
}
