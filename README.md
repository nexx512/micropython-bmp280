# micropython-bmp280

BMP280 environmental sensor driver for micropython using the I²C interface.

## Dependencies

* micropython
* machine

## Usage

Create a micropython I2C Object and pass it to the constructor.
Then initialize the device with `BMP280.initialize()`. Filter and oversampling parameters
can be passed to the initialize method.
Then you can either start continuous measurements with `BMP280.start_periodic_measurement(standby_time)`
or trigger a single measurement using `BMP280.trigger_single_measurement(callback)`.
The callback is optional. The function will block until the measurements are available
if no callback is defined.

```python
from machine import Pin, I2C, Timer
from bmp280 import BMP280

i2c = i2c_0 = I2C(1, scl = Pin(11), sda = Pin(10), freq = 400000)
bmp280 = BMP280(i2c = i2c, sdo = 0)

bmp280.initialize()
bmp280.trigger_single_measurement(lambda temperature, pressue: print("Temperature: %s, Pressure: %s" % (temperature, pressure)))
```
