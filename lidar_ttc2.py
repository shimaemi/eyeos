import serial # uart
from time import sleep
from tf_luna import TFLuna
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice

ser = serial.Serial('/dev/serial0', 115200)
# we define a new function that will get the data from LiDAR and publish it
sample = 10 # set sample rate 5 / sec
t = 1 / sample # period

# Initialize the haptic sensor on a specific GPIO pin
haptic_sensor = PWMOutputDevice(pin=4)

def vibrate():
    haptic_sensor.on()
    sleep(.5)
    haptic_sensor.off()

def vibrate2():
    haptic_sensor.on()
    sleep(.5) 
    haptic_sensor.off()
    sleep(.5)
    haptic_sensor.on()
    sleep(.5)
    haptic_sensor.off()

def read_data():
    sleep(1)  # Sleep 1000ms
    range = 0 # 1 if object within 10 sec, 2 if within 5
    prev = 0
    while True:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()
            
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # python3
                curr = bytes_serial[2] + bytes_serial[3]*256 # centimeters
                if curr != prev:
                    ttc = curr * t / (prev - curr)
                if ttc <= 5 and ttc > 0 and range < 2: # send an alert every time we enter the danger zone
                    vibrate2()
                    range = 2
                elif ttc <= 10 and ttc > 5 and range < 1:
                    vibrate()
                    range = 1
                elif ttc > 10 and range > 0:
                    range = 0
                elif ttc <= 10 and ttc > 5 and range > 1:
                    range = 1
                else:
                    print("TTC:"+ str(ttc) + "sec")
                prev = curr 
                ser.reset_input_buffer()

def set_samp_rate(samp_rate=100):
    ##########################
    # change the sample rate
    samp_rate_packet = [0x5a,0x06,0x03,samp_rate,00,00] # sample rate byte array
    ser.write(samp_rate_packet) # send sample rate instruction
    return
                
if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        set_samp_rate(sample) 
        read_data()
    except KeyboardInterrupt:
        if ser != None:
            ser.close()
            print("program interrupted by the user")
