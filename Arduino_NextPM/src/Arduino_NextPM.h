#ifndef Arduino_NextPM
#define Arduino_NextPM

#include "Arduino.h"
#include <HardwareSerial.h>
#include "Arduino_NextPM.h"
#include <string>

struct NextPMState {
    uint8_t stateBitmask;
    String stateDescriptorEnabled;
    String stateDescriptorDisabled;
    bool stateEnabled;

    NextPMState(uint8_t _stateBitmask, String _stateDescriptorEnabled, String _stateDescriptorDisabled, bool _stateEnabled) {
        stateBitmask = _stateBitmask;
        stateDescriptorEnabled = _stateDescriptorEnabled;
        stateDescriptorDisabled = _stateDescriptorDisabled;
        stateEnabled = _stateEnabled;
    }

    NextPMState() {

    }

    bool checkIfPresent(uint8_t stateCodeByte) {
        stateEnabled = (stateBitmask & stateCodeByte) != 0;
        return stateEnabled;
    }

    bool isTrue() {
        return stateEnabled;
    }

    String getString() {
        return (stateEnabled) ? stateDescriptorEnabled : stateDescriptorDisabled;
    }
};

struct MeasurementMode {
    String modeName;
    uint8_t modeCode;
    int avgPeriodSeconds;
    int transmitPeriodSeconds;

    MeasurementMode(String _modeName, uint8_t _modeCode, int _avgPeriodSeconds, int _transmitPeriodSeconds) {
        modeName = _modeName;
        modeCode = _modeCode;
        avgPeriodSeconds = _avgPeriodSeconds;
        transmitPeriodSeconds = _transmitPeriodSeconds;
    }
};

struct Measurement {
    int pcs_per_litre = 0;
    float density = 0.0;

    void calculateValues(uint8_t pcsByte1, uint8_t pcsByte2, uint8_t densityByte1, uint8_t densityByte2) {
        pcs_per_litre = pcsByte1 << 8 | pcsByte2;
        density = (densityByte1 << 8 | densityByte2) / 10;
    }
};

class NextPM 
{
    private:
        // Default "address" NextPM will respond to
        uint8_t ADDR = 0x81;

        // Set to true when initialisation function is run
        // Currently this is never reset to false after initialisation
        bool initialised = false;

        // Reference to HardwareSerial object - Tested with ESP32 only using Serial(1) on D22 & D23
        HardwareSerial *SENS_SERIAL;

        // Will hold the latest state code sent by the device
        uint8_t stateCode = 0x00;

        // String to hold list of errors/state descriptors, useful for debugging
        String allStateStrings;
        String activeStateStrings;

        // All states held in this array
        //NextPMState allStates[8];

        // State items which encode all required information about each bit in the state code byte
        NextPMState sleepEnabled = NextPMState (0x01, "SLEEP ENABLED", "SLEEP DISABLED", false);
        NextPMState degradedError = NextPMState (0x02, "SENSOR DEGRADED", "SENSOR OK", false);
        NextPMState notReady = NextPMState (0x04, "SENSOR NOT READY", "SENSOR READY", false);
        NextPMState highHumidityError = NextPMState (0x08, "HUMIDITY EXCEEDS LIMITS", "HUMIDITY OK", false);
        NextPMState tempHumidityError = NextPMState (0x10, "TEMP/HUMIDITY SENSOR ERROR", "TEMP/HUMIDITY SENSOR OK", false);
        NextPMState fanError = NextPMState (0x20, "FAN ERROR", "FAN OK", false);
        NextPMState memoryError = NextPMState (0x40, "INTERNAL MEMORY ERROR", "INTERNAL MEMORY OK", false);
        NextPMState laserError = NextPMState (0x80, "LASER ERROR", "LASER OK", false);

        // Current sensor measurement mode
        MeasurementMode* currentMode;

        bool noMeasurementAvailable = true;

        Measurement PM_1;
        Measurement PM_2_5;
        Measurement PM_10;

        float rawTemperatureCelsius = 0.0;
        float rawRelativeHumidityPercentage = 0.0;

        float correctedTemperatureCelsius = 0.0;
        float correctedRelativeHumidityPercentage = 0.0;

        // Stores the firmware version as requested upon initialisation function call
        uint8_t firmwareVersion = 0x00;

        // Array to to store bytes that will be sent / bytes that were last send as a command
        // All commands are three bytes long starting with the address
        uint8_t storedCommand[3] = {ADDR, 0x00, 0x00};

        // Stores the latest checksum success/failure result
        bool checksumError = false;

        // Array to store bytes recieved in order in the latest frame from the sensor.
        // Length of array stored alongside array during getResponse
        // Maximum frame is 16 bytes.
        uint8_t sensorResponse[16];
        uint8_t responseLength = 0;

        void sendStoredCommand();
        void readResponse();

        void setCommandChecksum();
        bool checkResponseChecksum();

        uint8_t getSensorState();
        void updateSensorState(uint8_t state);

    public:

        NextPM(uint8_t sensorAddress);

        bool init(HardwareSerial *SensorSerial, int rx, int tx);

        bool isReady();

        void doSleep();
        void doWake();

        void updateParticulateData(MeasurementMode newSensorMode);
        void updateTemperatureHumidityData();

        float PM_1_Density() { return PM_1.density; };
        float PM_2_5_Density() { return PM_2_5.density; };
        float PM_10_Density() { return PM_10.density; };

        float PM_1_Count() { return PM_1.pcs_per_litre; };
        float PM_2_5_Count() { return PM_2_5.pcs_per_litre; };
        float PM_10_Count() { return PM_10.pcs_per_litre; }

        float getTemperature() { return correctedTemperatureCelsius; };
        float getHumidity() { return correctedRelativeHumidityPercentage; };

        bool readyToMeasure();

        uint8_t getStateCode();
        String getAllStateStrings();
        String getActiveStateStrings();

        bool measurementIsValid();
        bool previousChecksumError();

        // All the sensor measurement mode structs
        // includes descriptions,
        MeasurementMode OFF{"NOT SAMPLING", 0x00, 0, 0};
        MeasurementMode FAST{"SECOND", 0x11, 10, 1};
        MeasurementMode MEDIUM{"TEN SECONDS", 0x12, 60, 10};
        MeasurementMode SLOW{"SIXTY SECONDS", 0x13, 900, 60};

        
};

#endif