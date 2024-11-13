import serial # uart
import time
from tf_luna import TFLuna
import RPi.GPIO as GPIO
ser = serial.Serial('/dev/serial0', 115200)

lid_samp = 10 # set sample rate 
t = 1 / lid_samp # period

vibration_pin = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(vibration_pin, GPIO.OUT)

def vibrate():
    GPIO.output(vibration_pin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(vibration_pin, GPIO.LOW)

def vibrate2():
    GPIO.output(vibration_pin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(vibration_pin, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(vibration_pin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(vibration_pin, GPIO.LOW)

def read_lidar():
    i = 2
    while i > 0:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()
            
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # python3
                curr = bytes_serial[2] + bytes_serial[3]*256 # centimeters
                if i == 2:
                    prev = curr 
                ser.reset_input_buffer()
                i = i - 1
    lid_ttc = 0
    if curr != prev:
        lid_ttc = curr * t / (prev - curr)
    return lid_ttc
                
if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        set_lid_samp(lid_samp)
        range = 0 # 1 if object within 10 sec, 2 if within 5
        while(1)
            lid_ttc = read_lidar()
            if lid_ttc <= 5 and lid_ttc > 0 and range < 2: # send an alert every time we enter the danger zone
                vibrate2()
                range = 2
            elif lid_ttc <= 10 and lid_ttc > 5 and range < 1:
                vibrate()
                range = 1
            elif lid_ttc > 10 and range > 0:
                range = 0
            elif lid_ttc <= 10 and lid_ttc > 5 and range > 1:
                range = 1
            else:
                pass
    except KeyboardInterrupt:
        if ser != None:
            ser.close()
            GPIO.cleanup()
            print("program interrupted by the user")