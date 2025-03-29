#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>

// Configuration constants
#define SLAVE_ADDRESS 0x04            // I2C address for this Arduino when acting as slave
#define BNO055_SAMPLE_DELAY_MS 100    // Delay between sensor readings (10Hz sample rate)

// Initialize BNO055 sensor object with I2C address 0x55
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Structure to hold IMU orientation data
// Values are stored as fixed-point integers (actual value * 100)
// to preserve 2 decimal places of precision over I2C
struct ImuData {
    int16_t roll;    // Range: -18000 to 18000 (-180.00° to 180.00°)
    int16_t pitch;   // Range: -9000 to 9000 (-90.00° to 90.00°)
    int16_t yaw;     // Range: 0 to 36000 (0.00° to 360.00°)
};

// Two buffers for double buffering
volatile ImuData buffer1;
volatile ImuData buffer2;
// Flag to track which buffer is being written to
volatile bool writing_to_first = true;

// Function prototype for I2C request handler
void sendData();

void setup() {
    // Initialize I2C communication as a slave device
    Wire.begin(SLAVE_ADDRESS);
    Wire.onRequest(sendData);  // Register request handler function
    
    // Initialize serial communication for debugging
    Serial.begin(9600);
    Serial.println("Starting BNO055 initialization...");
    
    // Initialize BNO055 sensor with error checking
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055!");
        Serial.println("Check your wiring or I2C address.");
        while (1);  // Halt if sensor initialization fails
    }
    
    Serial.println("BNO055 initialized successfully!");
    delay(1000);  // Allow time for sensor to stabilize
    
    // Enable external crystal for better accuracy
    bno.setExtCrystalUse(true);
    Serial.println("External crystal enabled");
}

void loop() {
    sensors_event_t event;
    bno.getEvent(&event);  // Get current sensor readings
    
    // Convert floating-point angles to fixed-point integers
    // Multiply by 100 to preserve 2 decimal places of precision
    // Note: BNO055 returns:
    //   x as yaw (0° to 360°)
    //   y as pitch (-90° to 90°)
    //   z as roll (-180° to 180°)
    if (writing_to_first) {
        buffer1.roll = (int16_t)(event.orientation.z * 100);
        buffer1.pitch = (int16_t)(event.orientation.y * 100);
        buffer1.yaw = (int16_t)(event.orientation.x * 100);
    } else {
        buffer2.roll = (int16_t)(event.orientation.z * 100);
        buffer2.pitch = (int16_t)(event.orientation.y * 100);
        buffer2.yaw = (int16_t)(event.orientation.x * 100);
    }
    
    // Atomic operation: switch buffers
    writing_to_first = !writing_to_first;
    
    // Print raw integer values for debugging
    Serial.print("Raw values - Roll: ");
    Serial.print(event.orientation.z * 100);
    Serial.print(" Pitch: ");
    Serial.print(event.orientation.y * 100);
    Serial.print(" Yaw: ");
    Serial.println(event.orientation.x * 100);
    
    // Print original floating-point values for comparison
    Serial.print("Float values - Roll: ");
    Serial.print(event.orientation.z);
    Serial.print(" Pitch: ");
    Serial.print(event.orientation.y);
    Serial.print(" Yaw: ");
    Serial.println(event.orientation.x);
    
    delay(BNO055_SAMPLE_DELAY_MS);  // Wait before next reading
}

void sendData() {
    // Buffer to hold the six bytes of IMU data (2 bytes each for roll, pitch, yaw)
    byte buffer[6];
    
    // Read from the buffer that's NOT being written to
    const volatile ImuData& data = writing_to_first ? buffer2 : buffer1;
    
    // Pack 16-bit integers into bytes for transmission
    // For each value, first byte is MSB (>>8), second is LSB (&0xFF)
    buffer[0] = (data.roll >> 8) & 0xFF;    // Roll MSB
    buffer[1] = data.roll & 0xFF;           // Roll LSB
    buffer[2] = (data.pitch >> 8) & 0xFF;   // Pitch MSB
    buffer[3] = data.pitch & 0xFF;          // Pitch LSB
    buffer[4] = (data.yaw >> 8) & 0xFF;     // Yaw MSB
    buffer[5] = data.yaw & 0xFF;            // Yaw LSB
    
    // Print byte values in hexadecimal for debugging
    Serial.print("Sending bytes: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Send all 6 bytes over I2C
    Wire.write(buffer, 6);
}