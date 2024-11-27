from tf_luna import TFLuna
import lgpio
import time

# Set up GPIO
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, 18)

# Function to vibrate the motor
def vibrate_motor(duration):
    lgpio.gpio_write(h, 18, 1)
    time.sleep(duration)
    lgpio.gpio_write(h, 18, 0)

try:
    while True:
        vibrate_motor(1)  # Vibrate for 1 second
        time.sleep(2)     # Wait for 2 seconds
except KeyboardInterrupt:
    pass
finally:
    lgpio.gpiochip_close(h)