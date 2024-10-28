import serial # uart
import time
ser = serial.Serial("/dev/ttyS0", 115200)
# we define a new function that will get the data from LiDAR and publish it
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
                ttc = curr * .01 / (prev - curr) # .01 = time between two measurements in seconds, 1 / framerate (100hz default)
                velLidar = (prev - curr) / .01
                print("TTC:"+ str(ttc) + "sec")
                print("velocity:"+ str(velLidar) + "cm/sec")
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