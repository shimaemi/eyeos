import serial # uart
import time
import RPi.GPIO as GPIO
import gpiozero import PWMOutputDevice

# Initialize the haptic sensor on a specific GPIO pin
haptic_sensor = PWMOutputDevice(pin=4)

def vibrate():
    haptic_sensor.on()
    sleep(.1)
    haptic_sensor.off()

def vibrate2():
    haptic_sensor.on()
    sleep(.1)
    haptic_sensor.off()
    sleep(.1)
    haptic_sensor.on()
    sleep(.1)
    haptic_sensor.off()
                
if __name__ == "__main__":
    try:
        while(1):
            print("warning 1")
            vibrate()
            sleep(2)
            print("warning 2")
            vibrate2()
            sleep(2)

    except KeyboardInterrupt:
            GPIO.cleanup()
            print("program interrupted by the user")