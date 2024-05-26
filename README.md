# PDL_Tilt_Sensor

PDL_Tilt_Sensor is an Arduino library that allows you to detect tilt using the LSM6DS3 sensor. This library provides configurable thresholds, callback functions, and FreeRTOS integration for real-time processing.

## Features

- Detect tilt using the LSM6DS3 sensor
- Configurable thresholds for vertical detection
- Callback functions for tilt and level events
- FreeRTOS integration for real-time processing

## Installation

To use this library, you need to install the following dependencies:

1. [Seeed_Arduino_LSM6DS3](https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3)
2. [MovingAverage](https://github.com/pilotak/MovingAverage.git)

### Installing Dependencies

- **Seeed_Arduino_LSM6DS3**
  - Download or clone the library from [https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3](https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3).
  - Unzip the library and place it in the `libraries` folder of your Arduino sketchbook.

- **MovingAverage**
  - Download or clone the library from [https://github.com/pilotak/MovingAverage.git](https://github.com/pilotak/MovingAverage.git).
  - Unzip the library and place it in the `libraries` folder of your Arduino sketchbook.

## Usage

Include the library in your sketch and initialize it.

## API

### `PDL_Tilt_Sensor`

#### Constructor

```cpp
PDL_Tilt_Sensor();
```
Creates an instance of the tilt sensor.

#### Methods

- `void init(uint32_t priority = 2);`
  Initializes the sensor with the specified priority.
- `void calibrate();`
  Calibrates the sensor.
- `float getAngleX() const;`
  Returns the angle in the X direction.
- `float getAngleY() const;`
  Returns the angle in the Y direction.
- `bool isVertical() const;`
  Checks if the sensor is in a vertical position.
- `void setDebugStatus(uint8_t status);`
  Sets the debug status.
- `void setVerticalThresholds(float x_lower, float x_upper, float y_lower, float y_upper);`
  Sets the thresholds for vertical detection.
- `void setLoopDelay(uint32_t delay_ms);`
  Sets the loop delay in milliseconds.
- `void setSampleSize(uint16_t samples);`
  Sets the sample size for the moving average filter.
- `void setTiltedCallback(IMUEventCallback callback);`
  Sets the callback function for tilt detection.
- `void setLevelCallback(IMUEventCallback callback);`
  Sets the callback function for level detection.

## License

This library is licensed under the MIT License. See [LICENSE](LICENSE) for more details.

## Contributing

Feel free to submit issues or pull requests for any bugs or feature requests.

## Credits

Developed by Xuteng Lin.
