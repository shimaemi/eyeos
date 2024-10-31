# sensor.py
import serial  # type: ignore
import time

class TFLuna:
    # port = serial port TF-Luna is connected to (~AMA0)
    # baudrate = communication speed default is set
    def __init__(self, port, baudrate = 115200):
        self.ser = serial.Serial(port, baudrate, timeout = 0)

    def set_sample(self, sample = 100):
        # Change the sample rate
        sample_packet = [0x5a,0x06,0x03,sample,00,00] # Sample rate byte array
        self.ser.write(sample_packet) # Send sample rate instruction
        time.sleep(0.1) # Wait or change to take effect
        return  

    def read_distance(self):
        if not self.ser.is_open:
            print("Serial port is not open.")
            return None
        while True:
            counter = self.ser.in_waiting    # Counts the number of bytes of the serial port
            if counter > 8:
                data = self.ser.read(9) # Read 9 bytes from the sensor
                self.ser.reset_input_buffer() # Resets buffer
                if data[0]  == 0x59 and data[1] == 0x59:    # Checking for the header bytes
                    distance = data[2] + data[3] * 256  # Calculates distance
                    return distance
        return None

    def read_strength(self):    # Signal strength
        if not self.ser.is_open:
            print("Serial port is not open.")
            return None
        while True:
            counter = self.ser.in_waiting    # Counts the number of bytes of the serial port
            if counter > 8:
                data = self.ser.read(9) # Read 9 bytes from the sensor
                self.ser.reset_input_buffer() # Resets buffer
                if data[0]  == 0x59 and data[1] == 0x59:    # Checking for the header bytes
                    strength = data[4] + data[5] * 256   # Calculates signal strength 
                    return strength
        return None

    def read_temperature(self):
        if not self.ser.is_open:
            print("Serial port is not open.")
            return None
        while True:
            counter = self.ser.in_waiting    # Counts the number of bytes of the serial port
            if counter > 8:
                data = self.ser.read(9) # Read 9 bytes from the sensor
                self.ser.reset_input_buffer() # Resets buffer
                if data[0]  == 0x59 and data[1] == 0x59:    # Checking for the header bytes
                    temperature = data[6] + data[7] * 256   # Calculates temperature
                    temperature = (temperature/8.0) - 256.0 # Temperature scaling and offset
                    return temperature
        return None       
    
    def convert_distance(self, distance, unit):
        # Converts distance to a specified unit
        if unit == 'mm':
            return distance * 10
        elif unit == 'cm':
            return distance
        elif unit == 'inch':
            return distance / 2.54
        elif unit == 'ft':
            return distance / 30.48
        elif unit == 'm':
            return distance / 100.0
        else:
            raise ValueError("Unsupported unit, Choose from 'mm', 'cm', 'inch', 'ft', 'm'. ")

    def convert_temperature(self, temperature, unit):
        # Convert temperature to the specified unit.
        if unit == 'C':
            return temperature
        elif unit == 'F':
            return (temperature * 9/5) + 32
        elif unit == 'K':
            return temperature + 273.15
        else:
            raise ValueError("Unsupported unit. Choose from 'C', 'F', 'K'.")
        
    def start_ttc_read():
        time.sleep(1)  # Sleep 1000ms
        range = 0 # 1 if object within 10 sec, 2 if within 5
        prev = 0
        while True:
            counter = self.ser.in_waiting # count the number of bytes of the serial port
            if counter > 8:
                bytes_serial = self.ser.read(9)
                self.reset_input_buffer()
            
                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # python3
                    curr = self.read_distance() # centimeters
                    if curr != prev:
                        ttc = curr * 1 / (prev - curr) # .01 = time between two measurements in seconds, 1 / framerate (100hz default)
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
                    else:
                        print("TTC:"+ str(ttc) + "sec")
                    prev = curr 
                    self.ser.reset_input_buffer()    
 
    def print_distance(self, unit = 'cm'):
        # Read and print the distance in the specified unit
        distance = self.read_distance() 
        if distance is not None:
            converted_distance = self.convert_distance(distance, unit)
            print(f"Distance: {converted_distance:.2f} {unit}")
        else:
            print("Failed to read distance.")

    def print_strength(self):
        # Read and print the signal strength 
        strength = self.read_strength()
        if strength is not None:
            print(f"Signal Strength: {strength}")
        else:
            print("Failed to read signal strength.")

    def print_temperature(self, unit = 'C'):
        # Read and print the temperature in the specified unit
        temperature = self.read_temperature()
        if temperature is not None:
            converted_temperature = self.convert_temperature(temperature, unit)
            print(f"Temperature: {converted_temperature:.2f} {unit}")
        else:
            print("Failed to read temperature.")

    def close(self):
        self.ser.close()
