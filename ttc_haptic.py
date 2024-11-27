from tf_luna import TFLuna
import lgpio
import time

# Set up GPIO
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, 18)

try:
    while True:
        ttc_value = sensor.print_ttc()
        if ttc_value is not None:
            if ttc_value <= 5:
                lgpio.tx_pwm(h, 18, 1000, 1000000)  # Strong vibration
            elif ttc_value <= 10:
                lgpio.tx_pwm(h, 18, 500, 1000000)  # Medium vibration
            else:
                lgpio.tx_pwm(h, 18, 0, 0)  # No vibration

except KeyboardInterrupt:
    pass
finally:
    lgpio.tx_pwm(h, 18, 0, 0)  # Stop PWM
    lgpio.gpiochip_close(h)

