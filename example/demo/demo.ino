#include <Arduino.h>
#include "PDL_Tilt_Sensor.h"

PDL_Tilt_Sensor imu;

void onTilted()
{
    Serial.println("Device is tilted");
}

void onLevel()
{
    Serial.println("Device is level");
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

    // Set debug status to display angles
    imu.setDebugStatus(IMU_DEBUG_STATUS_ANGLE);

    // Set vertical thresholds for X and Y directions
    imu.setVerticalThresholds(-10, 10, -10, 10);

    // Set loop delay
    imu.setLoopDelay(100);

    // Set the tilt and level callbacks
    imu.setTiltedCallback(onTilted);
    imu.setLevelCallback(onLevel);

    Serial.println("IMU setup complete.");
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == 'p')
        {
            Serial.println("Pausing IMU...");
            imu.pause();
        }
        else if (c == 'r')
        {
            Serial.println("Resuming IMU...");
            imu.resume();
        }
        else
        {
            int num = c - '0';
            imu.setDebugStatus(num);
        }
        while (Serial.available())
        {
            Serial.read(); // Clear the buffer
        }
    }
    delay(200);
}
