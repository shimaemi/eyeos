import serial # uart
import time
ser = serial.Serial("/dev/ttyS0", 115200)
# we define a new function that will get the data from LiDAR and publish it
range = 0 
# 1 if object within 10 sec, 2 if within 5
prev = 0
def read_data():
    time.sleep(1)  # Sleep 1000ms
    while True:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()
            
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # python3
                curr = bytes_serial[2] + bytes_serial[3]*256 # centimeters
                if curr != prev:
                    ttc = curr * .01 / (prev - curr) # .01 = time between two measurements in seconds, 1 / framerate (100hz default)
                if ttc <= 5 and ttc > 0 and range < 2: # send an alert every time we enter the danger zone
                    print("est TTC: within 5 sec")
                    range = 2
                elif ttc <= 10 and ttc > 5 and range < 1:
                    print("est TTC: within 10 sec")
                    range = 1
                elif ttc > 10 and range > 0:
                    range = 0
                elif ttc <= 10 and ttc > 5 and range > 1:
                    range = 1
                prev = curr 
                ser.reset_input_buffer()
                
if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        read_data()
    except KeyboardInterrupt:
        if ser != None:
            ser.close()
            print("program interrupted by the user")
