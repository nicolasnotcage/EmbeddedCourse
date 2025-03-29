# EmbeddedCourse

A collection of embedded systems projects written in C++ for an academic course on real-time embedded development. Each project showcases a different area of low-level embedded work, including interrupt handling, sensor integration, data logging, and real-time processing.

These projects were developed using various microcontrollers (Arduino Uno and ESP8266) and peripherals (IR sensors, BME280, BNO055, SD cards). All code is original and designed to demonstrate practical knowledge of embedded programming concepts.

---

## Projects

### 1. IMU Double-Buffered I2C Streaming (`imu_project.cpp`)
**Description:**
- Interfaces with an Adafruit BNO055 IMU sensor over I2C
- Implements **double buffering** to avoid race conditions between data production and transmission
- Converts float IMU data into scaled integers for efficient transmission
- Acts as an I2C slave device, transmitting sensor data to a master

**Key Concepts:**
- I2C communication
- Double buffering
- Fixed-point encoding
- Real-time sensor sampling

**Hardware:**
- Adafruit BNO055
- Arduino-compatible board

---

### 2. Temperature Logger with Interrupt-Based Serial Dump (`temperature_logging.cpp`)
**Description:**
- Reads temperature from a BME280 sensor
- Logs sensor values to SD card every second using a timer interrupt
- Allows dumping log contents to serial monitor when a button is pressed

**Key Concepts:**
- Timer interrupts
- SD card file I/O
- Button debounce and edge detection
- Sensor integration

**Hardware:**
- BME280 sensor
- SD card module
- Push button
- Arduino Uno

---

### 3. Propeller RPM Counter using IR (`optical_tachometer.cpp`)
**Description:**
- Uses an IR emitter/receiver pair to count propeller blade passes
- Calculates **revolutions per minute (RPM)** every second using hardware interrupts
- Controls motor speed and direction
- Toggle system state via button

**Key Concepts:**
- External interrupts
- Timer-based measurement
- Real-time signal counting
- GPIO motor control

**Hardware:**
- IR emitter and receiver
- Motor driver
- Push button
- Arduino Uno

---

## License
MIT License

## Author
Nicolas Miller
