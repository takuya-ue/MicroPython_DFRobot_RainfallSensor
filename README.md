# DFRobot Rainfall Sensor Library for Raspberry Pi Pico (MicroPython)

日本語版は[こちら](README-ja.md)

## overview
A MicroPython version of the Python library for DFRobot's Rainfall Sensor. Operation has been confirmed on Raspberry Pi Pico. It only supports I2C.

## Example of use
```python
import time
from machine import Pin
from MicroPython_DFRobot_RainfallSensor import *

rain_sensor = DFRobot_RainfallSensor_I2C(sda=Pin(0), scl=Pin(1))
while (rain_sensor.begin() == False):
   print("Sensor initialize failed!!")
   time.sleep(1)

print("Sensor initialize successfully")
accumulated_rainfall = rain_sensor.get_rainfall()
one_hour_rainfall = rain_sensor.get_rainfall_time(1)
````

## License
MIT license

## Acknowledgments
This library uses the [DFRobot_RainfallSensor](https://github.com/DFRobot/DFRobot_RainfallSensor) repository.
