# Raspberry Pi Pico (MicroPython)用 DFRobot Rainfall Sensor Library

## 概要
DFRobotのRainfall SensorのためのPythonライブラリのMicroPython版です。Raspberry Pi Picoで動作を確認しています。I2Cのみに対応しています。

## 使用例
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
```

## ライセンス
MITライセンス

## 謝辞
このライブラリは[DFRobot_RainfallSensor](https://github.com/DFRobot/DFRobot_RainfallSensor)リポジトリを使用しています。
