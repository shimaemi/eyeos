# lidar_sensor.py
import serial  # type: ignore
import time

class TFLuna:
    def __init__(self, baudrate=115200):
        # Initialize the sensor with the default serial port '/dev/serial0'
        self.ser = serial.Serial('/dev/serial0', baudrate, timeout=0)  # Default port set here

    def read_distance(self):
        """Read the distance from the TF Luna sensor"""
        if not self.ser.is_open:
            print("Serial port is not open.")
            return None
        while True:
            counter = self.ser.in_waiting  # Counts the number of bytes in the serial port
            if counter > 8:
                data = self.ser.read(9)  # Read 9 bytes from the sensor
                self.ser.reset_input_buffer()  # Reset buffer
                if data[0] == 0x59 and data[1] == 0x59:  # Check for valid header bytes
                    distance = data[2] + data[3] * 256  # Calculate distance (in cm)
                    return distance
        return None

    def convert_distance(self, distance, unit):
        """Convert distance to the specified unit"""
        if unit == 'cm':
            return distance
        elif unit == 'inch':
            return distance / 2.54
        elif unit == 'm':
            return distance / 100.0
        else:
            raise ValueError("Unsupported unit. Use 'cm', 'inch', or 'm'.")

    def close(self):
        """Close the serial connection"""
        self.ser.close()
