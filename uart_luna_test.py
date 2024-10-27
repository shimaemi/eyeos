import serial # uart
import time
ser = serial.Serial("/dev/ttyS0", 115200)
# we define a new function that will get the data from LiDAR and publish it
range = 0 
# 1 if object within 100 cm, 2 if within 50
def read_data():
    time.sleep(.01)  # Sleep 10ms
    while True:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()
            
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # python3
                distance = bytes_serial[2] + bytes_serial[3]*256 # centimeters
                # strength = bytes_serial[4] + bytes_serial[5]*256 # Amp value over 32768 indicates ambient light overexposure
                if distance < 50 and range < 2
                    print("Warning: within 50 cm")
                    range = 2
                elif distance < 100 and distance > 50 and range < 1
                    print("Warning: within 100 cm")
                    range = 1
                elif distance > 100 and range > 0
                    range = 0
                elif distance > 50 and distance < 100 and range > 1
                    range = 1
                # print("Strength:" + str(strength))
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

