#include "PDL_Tilt_Sensor.h"
#include "Adafruit_TinyUSB.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

#define DEFAULT_VERTICAL_THRESHOLD_DEGREES 10
#define DEFAULT_TASK_PRIORITY 2
#define DEFAULT_LOOP_DELAY_MS 100
#define TASK_STACK_SIZE 1024
#define DEFAULT_ANGLE_X_OFFSET 90
#define DEFAULT_ANGLE_Y_OFFSET 90

PDL_Tilt_Sensor::PDL_Tilt_Sensor()
    : imu(I2C_MODE, 0x6A), angle_x(0), angle_y(0),
      x_lower_threshold(-DEFAULT_VERTICAL_THRESHOLD_DEGREES),
      x_upper_threshold(DEFAULT_VERTICAL_THRESHOLD_DEGREES),
      y_lower_threshold(-DEFAULT_VERTICAL_THRESHOLD_DEGREES),
      y_upper_threshold(DEFAULT_VERTICAL_THRESHOLD_DEGREES),
      is_vertical(false), was_vertical(false),
      loop_delay(DEFAULT_LOOP_DELAY_MS), debug_status(IMU_DEBUG_STATUS_NONE),
      ax_offset(DEFAULT_ANGLE_X_OFFSET), ay_offset(DEFAULT_ANGLE_Y_OFFSET), imu_task_handle(NULL),
      tilted_callback(nullptr), level_callback(nullptr) {}

void PDL_Tilt_Sensor::init(uint32_t priority)
{
    if (imu.begin() != 0)
    {
        Serial.println("IMU error");
    }
    else
    {
        Serial.println("IMU OK!");
    }

    xTaskCreate(imuTask, "IMU_TASK", TASK_STACK_SIZE, this, priority, &imu_task_handle);
}

void PDL_Tilt_Sensor::deinit()
{
    if (imu_task_handle != NULL)
    {
        vTaskDelete(imu_task_handle);
        imu_task_handle = NULL;
    }
}

void PDL_Tilt_Sensor::pause()
{
    if (imu_task_handle != NULL)
    {
        vTaskSuspend(imu_task_handle);
    }
}

void PDL_Tilt_Sensor::resume()
{
    if (imu_task_handle != NULL)
    {
        vTaskResume(imu_task_handle);
    }
}

void PDL_Tilt_Sensor::imuTask(void *pvParameters)
{
    PDL_Tilt_Sensor *instance = static_cast<PDL_Tilt_Sensor *>(pvParameters);
    instance->xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        instance->processIMUData();
        vTaskDelayUntil(&(instance->xLastWakeTime), pdMS_TO_TICKS(instance->loop_delay));
    }
}

void PDL_Tilt_Sensor::processIMUData()
{
    int16_t ax_raw = imu.readRawAccelX();
    int16_t ay_raw = imu.readRawAccelY();
    int16_t az_raw = imu.readRawAccelZ();

    float ax = ax_filter.add(ax_raw);
    float ay = ay_filter.add(ay_raw);
    float az = az_filter.add(az_raw);

    angle_x = atan2(sqrt(ay * ay + az * az), ax) * 180 / M_PI - ax_offset - 0.5 * (x_lower_threshold + x_upper_threshold);
    angle_y = atan2(sqrt(ax * ax + az * az), ay) * 180 / M_PI - ay_offset - 0.5 * (y_lower_threshold + y_upper_threshold);

    // rotate [0;0;1] vector by angle_x and angle_y
    float x2 = sin(angle_x * M_PI / 180 + angle_y * M_PI / 180);
    float y2 = -sin(angle_x * M_PI / 180);
    float z2 = cos(angle_x * M_PI / 180 + angle_y * M_PI / 180);

    azumith = atan2(y2, x2) * 180 / M_PI;
    azumith_magnitude = sqrt(x2 * x2 + y2 * y2) / sqrt(x2 * x2 + y2 * y2 + z2 * z2);

    // azumith = atan2(ay, ax) * 180 / M_PI;
    // azumith_magnitude = sqrt(ax * ax + ay * ay) / sqrt(ax * ax + ay * ay + az * az);

    checkTiltStatus();
    checkAzumith();

    switch (debug_status)
    {
    case IMU_DEBUG_STATUS_RAW:
        Serial.printf("ax:%6d, ay:%6d, az:%6d\n", ax_raw, ay_raw, az_raw);
        break;
    case IMU_DEBUG_STATUS_FILTERED:
        Serial.printf("ax:%6d, ay:%6d, az:%6d\n", ax, ay, az);
        break;
    case IMU_DEBUG_STATUS_ANGLE:
        Serial.printf("angle_x:%6.3f, angle_y:%6.3f, is_vertical:%d\n", angle_x, angle_y, is_vertical);
        break;
    case IMU_DEBUG_THRESHOLD:
        Serial.printf("angle_x:%6.3f, angle_y:%6.3f, is_vertical:%d\n", angle_x, angle_y, is_vertical);
        break;
    default:
        break;
    }
}

void PDL_Tilt_Sensor::calibrate()
{
    ax_offset = angle_x;
    ay_offset = angle_y;
}

void PDL_Tilt_Sensor::setDebugStatus(uint8_t status)
{
    debug_status = status;
}

void PDL_Tilt_Sensor::setVerticalThresholds(float x_lower, float x_upper, float y_lower, float y_upper)
{
    x_lower_threshold = x_lower;
    x_upper_threshold = x_upper;
    y_lower_threshold = y_lower;
    y_upper_threshold = y_upper;
}

void PDL_Tilt_Sensor::setLoopDelay(uint32_t delay_ms)
{
    loop_delay = delay_ms;
}

void PDL_Tilt_Sensor::setSampleSize(uint16_t samples)
{
    ax_filter.set_samples(samples);
    ay_filter.set_samples(samples);
    az_filter.set_samples(samples);
}

void PDL_Tilt_Sensor::setTiltedCallback(IMUEventCallback callback)
{
    tilted_callback = callback;
}

void PDL_Tilt_Sensor::setLevelCallback(IMUEventCallback callback)
{
    level_callback = callback;
}

void PDL_Tilt_Sensor::setAzumithUpdateCallback(IMUAzumithUpdateCallback callback)
{
    if (callback == nullptr)
    {
        azumith_update_callback = nullptr;
        return;
    }
    azumith_update_callback = callback;
}

float PDL_Tilt_Sensor::getAngleX() const
{
    return angle_x;
}

float PDL_Tilt_Sensor::getAngleY() const
{
    return angle_y;
}

bool PDL_Tilt_Sensor::isVertical() const
{
    return is_vertical;
}

float PDL_Tilt_Sensor::getAzumith() const
{
    return azumith;
}

float PDL_Tilt_Sensor::getAzumithMagnitude() const
{
    return azumith_magnitude;
}

void PDL_Tilt_Sensor::checkTiltStatus()
{
    bool currently_vertical = (angle_x > x_lower_threshold && angle_x < x_upper_threshold &&
                               angle_y > y_lower_threshold && angle_y < y_upper_threshold);

    if (currently_vertical != was_vertical)
    {
        was_vertical = currently_vertical;
        is_vertical = currently_vertical;
        if (currently_vertical && level_callback)
        {
            level_callback();
        }
        else if (!currently_vertical && tilted_callback)
        {
            tilted_callback();
        }
    }
}

void PDL_Tilt_Sensor::checkAzumith()
{
    if (azumith_update_callback)
    {
        AzumithCbsContext_t context = {
            .azumith = azumith,
            .azumith_magnitude = azumith_magnitude,
        };
        azumith_update_callback(&context);
    }
}
