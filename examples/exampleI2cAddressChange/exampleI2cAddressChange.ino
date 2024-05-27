/*
 * Copyright (c) 2023, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <Arduino.h>
#include <SensirionI2cSf06Lf.h>
#include <Wire.h>

// GPIO pins to which the IRQn pin of your sensors are connected
#define IRQN_PIN_SENSOR_A 4
#define IRQN_PIN_SENSOR_B 2

// I2C addresses to use for the sensors
#define I2C_ADDR_SENSOR_A 0x0A
#define I2C_ADDR_SENSOR_B 0x0B

// Define the sensor objects used for sensor communication after the address
// change
SensirionI2cSf06Lf sensorA;
SensirionI2cSf06Lf sensorB;

static char errorMessage[64];
static int16_t error;

void print_byte_array(uint8_t* array, uint16_t len) {
    uint16_t i = 0;
    Serial.print("0x");
    for (; i < len; i++) {
        Serial.print(array[i], HEX);
    }
}

/**
 * Perform soft reset for all sensor connected to I2C Bus.
 */
void i2c_soft_reset() {
    Wire.beginTransmission(0x00);
    size_t writtenBytes = Wire.write(0x06);
    uint8_t i2c_error = Wire.endTransmission();
}

/**
 * @brief Change the I2C address of the sensor.
 *
 * @details If you have more then one SLF3x sensor
 * connected to the same I2C bus you need to configure the sensor addresses as
 * all of them have the same default address.

 * Preconditions to call this method:
 * * Your sensor needs to be resetted (hard or soft reset, no commands sent
 *   after the reset)
 * * PIN1 of the sensor for which you want to change the address must be
 *   connected to a GPIO pin
 * * Note that only sensors with a serial number above 22xxxxxxxx have the IRQn
 *   pin
 *
 * @note The I2C address is not configured permanently, you need to run the
 * procedure after each restart of the sensor
 * (hard or soft reset depending on your sensor model).
 *
 * @param newI2cAddress the I2C address that shall be assigned to your sensor
 * @param sensorIrqPin GPIO pin number to which you have IRQn Pin (Pin nr 1) of
 *                     the sensor connected
 * @return error_code 0 on success, an error code otherwise (-1 if the
 *         confirmation pulse from the sensor could not be read)
 */
int16_t changeSensorAddress(TwoWire& wire, uint16_t newI2cAddress,
                            uint8_t sensorIrqPin) {
    uint8_t communication_buffer[5] = {0};
    int16_t localError = NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;

    // Send I2C address change command 0x3661 with the new I2C address as a
    // parameter (including CRC for the parameter)
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x3661, buffer_ptr, 5);
    txFrame.addUInt16(newI2cAddress);
    // Note that the command is sent to the default address 0x08 of the sensor
    localError = SensirionI2CCommunication::sendFrame(SLF3C_1300F_I2C_ADDR_08,
                                                      txFrame, wire);
    if (localError != NO_ERROR) {
        Serial.println("error sending address change command");
        errorToString(localError, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        Serial.println(
            "As there are multiple sensors attached initially listening on the same I2C address \
        the acknowledge might overlap and cause an error which you can ignore if the subsequent communication is successful.");
    }

    // set IRQN pin of one sensor to high for at least 150μs to confirm address
    // change only after this pulse has been sent the sensor actually accepts
    // the new I2C address sent before
    pinMode(sensorIrqPin, OUTPUT);
    digitalWrite(sensorIrqPin, HIGH);
    delayMicroseconds(500);
    // reset IRQn pin back to low state
    digitalWrite(sensorIrqPin, LOW);

    // switch mode to input and listen to the pulse the sensor
    // sends 1500μs after the address change command to confirm the new I2C
    // address
    // If your platform supports INPUT_PULLDOWN you should use it instead of
    // INPUT. This is e.g. the case for ESP32.
    pinMode(sensorIrqPin, INPUT);
    delayMicroseconds(500);
    uint8_t success = 0;
    uint16_t cnt = 0;
    while (success == 0 && cnt < 100) {
        cnt++;
        success = digitalRead(sensorIrqPin);
        delayMicroseconds(10);
    }
    if (success == 0) {
        // return error as sensor did not acknowledge address change
        return -1;
    }

    Serial.print("Sensor address changed to: 0x");
    if (newI2cAddress < 16) {
        Serial.print("0");
    }
    Serial.println(newI2cAddress, HEX);
    return NO_ERROR;
}

void readAndPrintSerial(SensirionI2cSf06Lf& sensor) {
    uint32_t productIdentifier = 0;
    uint8_t serialNumber[8] = {0};
    error = sensor.readProductIdentifier(productIdentifier, serialNumber, 8);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute readProductIdentifier(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
    Serial.print("productIdentifier: ");
    Serial.print(productIdentifier);
    Serial.print("\t");
    Serial.print("serialNumber: ");
    print_byte_array(serialNumber, 8);
    Serial.println();
}

void readAndPrintMeasurement(SensirionI2cSf06Lf& sensor) {
    float aFlow = 0.0;
    float aTemperature = 0.0;
    uint16_t aSignalingFlags = 0u;
    error = sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F,
                                       aFlow, aTemperature, aSignalingFlags);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute readMeasurementData(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
    Serial.print("aFlow: ");
    Serial.print(aFlow);
    Serial.print("\t");
    Serial.print("aTemperature: ");
    Serial.print(aTemperature);
    Serial.print("\t");
    Serial.print("aSignalingFlags: ");
    Serial.print(aSignalingFlags);
    Serial.println();
}

void setup() {
    error = NO_ERROR;

    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Wire.begin();

    // Make sure that sensors are in proper state to perform a address change by
    // doing a soft reset and not sending any other commands prior to the
    // address change procedure
    i2c_soft_reset();
    // SLF3x sensors need 25ms to start up after the reset
    delay(25);

    // Change address of the first sensor
    // Set IRQN_PIN_SENSOR_A to the GPIO pin number where you connected Pin 1
    // of your first sensor.
    error = changeSensorAddress(Wire, I2C_ADDR_SENSOR_A, IRQN_PIN_SENSOR_A);
    if (error != NO_ERROR) {
        Serial.print("Error changing sensor address: ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }

    // Change address of the first sensor
    // Set IRQN_PIN_SENSOR_B to the GPIO pin number where you connected Pin 1
    // of your second sensor.
    error = changeSensorAddress(Wire, I2C_ADDR_SENSOR_B, IRQN_PIN_SENSOR_B);
    if (error != NO_ERROR) {
        Serial.print("Error changing sensor address: ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }

    // Initialize first sensor
    Serial.println("Initialising sensor A");
    sensorA.begin(Wire, 0x0A);
    readAndPrintSerial(sensorA);
    error = sensorA.startH2oContinuousMeasurement();
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute startH2oContinuousMeasurement() "
                     "for sensor A: ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }

    // Initialize second sensor
    Serial.println("Initialising sensor B");
    sensorB.begin(Wire, 0x0B);
    readAndPrintSerial(sensorB);
    error = sensorB.startH2oContinuousMeasurement();
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute startH2oContinuousMeasurement() "
                     "for sensor B: ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
}

void loop() {
    delay(20);
    // Read out measurements for first sensor
    Serial.print("sensor A - ");
    readAndPrintMeasurement(sensorA);
    // Read out measurements for second sensor
    Serial.print("sensor B - ");
    readAndPrintMeasurement(sensorB);
}
