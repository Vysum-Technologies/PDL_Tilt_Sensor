#include <Arduino.h>
#include "PDL_Tilt_Sensor.h"

PDL_Tilt_Sensor imu;

void onAzimuthUpdate(PDL_Tilt_Sensor::AzimuthCbsContext_t *context)
{
    Serial.print("Azimuth: ");
    Serial.print(context->azimuth);
    Serial.print(" Magnitude: ");
    Serial.println(context->azimuth_magnitude);
}

void onTileted()
{
    Serial.println("Device is tilted,, enable azimuth callback");
    imu.setAzimuthUpdateCallback(onAzimuthUpdate);
}

void onLevel()
{
    Serial.println("Device is level, disable azimuth callback");
    imu.setAzimuthUpdateCallback(nullptr);
}

void setup()
{
    // Initialize Serial for debugging outputs
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial port to connect.

    Serial.println("Initializing system...");

    // Initialize the IMU
    imu.init();

    // Set debug status
    imu.setDebugStatus(PDL_Tilt_Sensor::DEBUG_NONE);

    // Set vertical thresholds for X and Y directions
    imu.setVerticalThresholds(-10, 10, -10, 10);

    // Set loop delay
    imu.setLoopDelay(100);

    // Set the tilt and level callbacks
    imu.setTiltedCallback(onTileted);
    imu.setLevelCallback(onLevel);
    imu.setAzimuthUpdateCallback(nullptr);

    Serial.println("IMU setup complete.");
}

void loop()
{
    yield();
}