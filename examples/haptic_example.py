import board
import busio
import adafruit_drv2605

i2c = busio.I2C(board.SCL, board.SDA)
drv = adafruit_drv2605.DRV2605(i2c)

#drv.sequence[0] = adafruit_drv2605.Effect(1)
drv.sequence[0] = adafruit_drv2605.Effect(89)
drv.play()

