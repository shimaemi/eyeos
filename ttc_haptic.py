from tf_luna import TFLuna
import lgpio
import time

# Set up GPIO
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, 18)

try:
    while True:
        for duty_cycle in range(0, 101, 10):
            lgpio.tx_pwm(h, 18, 100, duty_cycle)
            time.sleep(1)
        for duty_cycle in range(0, -1, -10):
            lgpio.tx_pwm(h, 18, 100, duty_cycle)
            time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    lgpio.tx_pwm(h, 18, 0, 0)  # Stop PWM
    lgpio.gpiochip_close(h)

