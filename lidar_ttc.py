import serial # uart
import time
from tf_luna import TFLuna
ser = serial.Serial('/dev/serial0', 115200)

lid_samp = 5 # set sample rate 5 / sec
t = 1 / lid_samp # period

def read_lidar():
    i = 2
    while i > 0:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()
            
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # python3
                curr = bytes_serial[2] + bytes_serial[3]*256 # centimeters
                if i == 2
                    prev = curr 
                ser.reset_input_buffer()
                i = i - 1
    lid_ttc = 0
    if curr != prev:
        lid_ttc = curr * t / (prev - curr)
    return lid_ttc

def set_lid_samp(self, lid_rate=5):
    ##########################
    # change the sample rate
    samp_rate_packet = [0x5a,0x06,0x03,lid_rate,00,00] # sample rate byte array
    ser.write(samp_rate_packet) # send sample rate instruction
    return
                
if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        set_lid_samp(lid_samp) 
        time.sleep(1)  # Sleep 1000ms
        range = 0 # 1 if object within 10 sec, 2 if within 5
        while(1)
            lid_ttc = read_lidar()
            if lid_ttc <= 5 and lid_ttc > 0 and range < 2: # send an alert every time we enter the danger zone
                print("est TTC: within 5 sec")
                range = 2
            elif lid_ttc <= 10 and lid_ttc > 5 and range < 1:
                print("est TTC: within 10 sec")
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
            print("program interrupted by the user")