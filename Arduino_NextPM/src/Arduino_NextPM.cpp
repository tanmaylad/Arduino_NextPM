#include "Arduino.h"
#include <HardwareSerial.h>
#include "Arduino_NextPM.h"
#include <string>

NextPM::NextPM(uint8_t sensorAddress = 0x81) {
    ADDR = sensorAddress;
}

bool NextPM::init(HardwareSerial *serObj, int rx, int tx) {
    currentMode = &OFF;
    SENS_SERIAL = serObj;
    SENS_SERIAL->begin(115200, SERIAL_8N1, rx, tx);
    delay(350);
    updateSensorState(getSensorState());
    return !checksumError;
}

void NextPM::sendStoredCommand() {
    storedCommand[0] = ADDR;
    setCommandChecksum();
    for (int i = 0; i < 3; i ++) {
        SENS_SERIAL->write(storedCommand[i]);
        delay(1);
    }
}

void NextPM::readResponse() {
    responseLength = 0;

    while (SENS_SERIAL->available()) {
        sensorResponse[responseLength] = SENS_SERIAL->read();
        responseLength += 1;
        if (responseLength > 16) {
            break;
        }
    }
}

bool NextPM::checkResponseChecksum() {
    int calculatedChecksum = 0;

    for (int i = 0; i < responseLength; i ++) {
        calculatedChecksum += sensorResponse[i];
    }

    checksumError = (calculatedChecksum % 0x100) != 0;

    return checksumError;
}

void NextPM::setCommandChecksum() {
    storedCommand[2] = (0x100 - storedCommand[1] - storedCommand[0]) % 0x100;
}

void NextPM::updateSensorState(uint8_t state) {
    
    allStateStrings = "";
    activeStateStrings = "";

    stateCode = state;

    if (checksumError) {
    allStateStrings += ("CHECKSUM INVALID - CHECK WIRING AND SENSOR\n");
    activeStateStrings += ("CHECKSUM INVALID - CHECK WIRING AND SENSOR\n");
    stateCode = 0xFF;
    }

    NextPMState *allStates[8];

    allStates[0] = &sleepEnabled;
    allStates[1] = &degradedError;
    allStates[2] = &notReady;
    allStates[3] = &highHumidityError;
    allStates[4] = &tempHumidityError;
    allStates[5] = &fanError;
    allStates[6] = &memoryError;
    allStates[7] = &laserError;

    for (int i = 0; i < 8; i ++) {
        if (allStates[i]->checkIfPresent(stateCode)) {
            activeStateStrings += allStates[i]->getString();
            activeStateStrings += "\n";
        }
        allStateStrings += allStates[i]->getString();
        allStateStrings += "\n";
    }
}

uint8_t NextPM::getSensorState() {
    storedCommand[1] = 0x16;
    sendStoredCommand();

    delay(350);

    readResponse();
    checkResponseChecksum();

    return sensorResponse[2];
}

void NextPM::doSleep() {

    updateSensorState(getSensorState());

    // Only sleep if not sleeping already
    if (!sleepEnabled.isTrue()) {
        storedCommand[1] = 0x15;
        sendStoredCommand();

        delay(350);

        readResponse();

        checkResponseChecksum();

        if (checksumError) {
            updateSensorState(getSensorState());;
        } else {
            updateSensorState(sensorResponse[2]);
        }

    };
}

void NextPM::doWake() {

    updateSensorState(getSensorState());

    // Only wake if not awake already
    if (sleepEnabled.isTrue()) {
        storedCommand[1] = 0x15;
        sendStoredCommand();

        delay(1000);

        updateSensorState(getSensorState());
    };

}

void NextPM::updateParticulateData(MeasurementMode newSensorMode) {

    updateSensorState(getSensorState());

    if (notReady.isTrue() || sleepEnabled.isTrue() || laserError.isTrue()) {
        return;
    }

    storedCommand[1] = newSensorMode.modeCode;
    sendStoredCommand();

    delay(350);

    readResponse();

    checkResponseChecksum();

    if (checksumError) {
        return;
    } else {
        updateSensorState(sensorResponse[2]);
    }

    if (sensorResponse[2] == 0x16) {
        noMeasurementAvailable = true;
        updateSensorState(getSensorState());
        return;
    } else {
        noMeasurementAvailable = false;
        currentMode = &newSensorMode;

        PM_1.calculateValues(sensorResponse[3], sensorResponse[4], sensorResponse[5], sensorResponse[6]);
        PM_2_5.calculateValues(sensorResponse[7], sensorResponse[8], sensorResponse[9], sensorResponse[10]);
        PM_10.calculateValues(sensorResponse[11], sensorResponse[12], sensorResponse[13], sensorResponse[14]);
    }
}

void NextPM::updateTemperatureHumidityData() {

    updateSensorState(getSensorState());

    if (notReady.isTrue() || tempHumidityError.isTrue()) {
        return;
    }

    storedCommand[1] = 0x14;
    sendStoredCommand();

    delay(350);

    readResponse();

    checkResponseChecksum();

    if (checksumError) {
        return;
    } else {
        updateSensorState(sensorResponse[2]);
    }

    if (sensorResponse[2] == 0x16) {
        noMeasurementAvailable = true;
        return;
    } else {
        noMeasurementAvailable = false;;
    }

    rawTemperatureCelsius = (sensorResponse[3] << 8 | sensorResponse[4]) / 100.0;
    rawRelativeHumidityPercentage = (sensorResponse[5] << 8 | sensorResponse[6]) / 100.0;

    correctedTemperatureCelsius = 0.9754 * rawTemperatureCelsius - 4.2488;
    correctedRelativeHumidityPercentage = 1.1768 * rawRelativeHumidityPercentage - 4.727;
}

bool NextPM::readyToMeasure() {

    updateSensorState(getSensorState());

    return !notReady.isTrue();
}

String NextPM::getAllStateStrings() {

    updateSensorState(getSensorState());

    return allStateStrings;
}

String NextPM::getActiveStateStrings() {

    updateSensorState(getSensorState());

    return activeStateStrings;
}

bool NextPM::previousChecksumError() {
    return checksumError;
}

bool NextPM::measurementIsValid() {
    return !noMeasurementAvailable;
}

uint8_t NextPM::getStateCode() {
    updateSensorState(getSensorState());
    return stateCode;
}
