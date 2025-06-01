# Documentation

**Sadeep Ariyarathna**  
[sadeepari@gmail.com](mailto:sadeepari@gmail.com)

---

## Table of Contents

1. [Wiring Diagram](#1-wiring-diagram)  
2. [Description](#2-description)  
3. [Variables](#3-variables)  
4. [Core Functions](#4-core-functions)  
5. [Program Flow](#5-program-flow)  
6. [MQTT Python Script](#6-mqtt-python-script)

---

## 1. Wiring Diagram

*(Diagram content not included in the original text.)*

---

## 2. Description

The program is capable of collecting data from the MCP9600 sensor and sending that data over MQTT using the SIM7600 SIM module. Data processing and transmission are managed via DMA within the F446RE. Temperature is collected according to a pre-set timer function. Measuring is switched to a faster frequency whenever the measured temperature exceeds a given threshold.

---

## 3. Variables

- **MQTT Variables**: Stores configuration for the MQTT broker like APN, host, port, topics, and connection status flags.  
- **SIM7600 Variables**: Stores AT command strings and response buffers.  
- **Temperature Reading Variables**: Holds the I2C address of the temperature sensor and stores temperature readings.  
- **I2C Callback Variables**: Manages data buffers for handling temperature data.

---

## 4. Core Functions

- `powerOn()`: Powers on the SIM7600 module.  
- `SIMTransmit(char cmd)`: Sends AT commands to the SIM7600 module and captures the response.  
- `startMQTT()`: Initializes and starts the MQTT connection with the broker. It first checks if the module is ready by sending AT commands, checks for network registration (`AT+CGREG?`), and finally starts the MQTT service.  
- `transmitMQTT()`: Publishes temperature data to the MQTT broker. It formats the data as a JSON-like string.  
- `endMQTT()`: *(Not used yet)* It is meant to properly disconnect and release the MQTT connection.  
- `mainMQTT()`: Manages the main MQTT loop, checking and maintaining the connection, and transmitting data.  
- `readTemperature()`: Reads temperature data from the sensor over I2C and stores it in a buffer.  
- `HAL_I2C_MasterRxCpltCallback()`: Callback function executed upon I2C data reception. Converts raw temperature data to a usable format and manages data buffers.  
- `HAL_TIM_PeriodElapsedCallback()`: Handles timer interrupts for periodic temperature data acquisition and manages the trigger for faster data collection if a temperature threshold is crossed.  
- `printBuffer()`: *(For debugging)* Prints the contents of the temperature data buffer.

---

## 5. Program Flow

- **Main Function**:  
  - Initializes the system and peripherals.  
  - Starts a timer for periodic interrupts.  
  - Continuously runs the `mainMQTT()` loop to maintain and transmit data over MQTT.

  > Before running the C program, the Python script should be run. (See next chapter)

- **Temperature Monitoring**:  
  - Periodically (based on timer interrupts) reads the temperature.  
  - If the temperature crosses a threshold, it triggers a faster sampling rate and expands the buffer size.  
  - Data is transmitted via MQTT if the connection is established.

---

## 6. MQTT Python Script

- Open CMD in the script directory.  
- Run the command:

  ```bash
  node app
