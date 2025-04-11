import serial  # type: ignore
import lgpio
import time

class TFLuna:
    def __init__(self, port, baudrate=115200, pwm_pin=18):
        # Initialize the sensor with the given port and baudrate
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.pwm_pin = pwm_pin
        self.gpio = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.gpio, self.pwm_pin)

    def read_distance(self):
        """Read the distance from the TF Luna sensor and ensure accurate results."""
        if not self.ser.is_open:
            print("Serial port is not open.")
            return None
        
        # Read and process data until valid information is received
        while True:
            counter = self.ser.in_waiting  # Counts the number of bytes in the serial port
            if counter > 8:
                data = self.ser.read(9)  # Read 9 bytes from the sensor
                self.ser.reset_input_buffer()  # Reset buffer to avoid stale data
                
                # Validate the header bytes and ensure data integrity
                if data[0] == 0x59 and data[1] == 0x59:  # Check for valid header bytes
                    distance = data[2] + data[3] * 256  # Calculate distance (in cm)
                    
                    # Verify that the distance value is within a reasonable range
                    if 0 < distance < 10000:  # Ensure distance is within a valid range (0-100m)
                        return distance
                    else:
                        print(f"Invalid distance reading: {distance} cm. Retrying...")
                else:
                    print("Invalid header bytes. Retrying...")
            time.sleep(0.1)  # Delay to avoid overloading the serial connection
        
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
import serial  # type: ignore
import lgpio
import time

class TFLuna:
    def __init__(self, port, baudrate=115200, pwm_pin=18):
        # Initialize the sensor with the given port and baudrate
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.pwm_pin = pwm_pin
        self.gpio = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.gpio, self.pwm_pin)

    def read_distance(self):
        """Read the distance from the TF Luna sensor and ensure accurate results."""
        if not self.ser.is_open:
            print("Serial port is not open.")
            return None
        
        # Read and process data until valid information is received
        while True:
            counter = self.ser.in_waiting  # Counts the number of bytes in the serial port
            if counter > 8:
                data = self.ser.read(9)  # Read 9 bytes from the sensor
                self.ser.reset_input_buffer()  # Reset buffer to avoid stale data
                
                # Validate the header bytes and ensure data integrity
                if data[0] == 0x59 and data[1] == 0x59:  # Check for valid header bytes
                    distance = data[2] + data[3] * 256  # Calculate distance (in cm)
                    
                    # Verify that the distance value is within a reasonable range
                    if 0 < distance < 10000:  # Ensure distance is within a valid range (0-100m)
                        return distance
                    else:
                        print(f"Invalid distance reading: {distance} cm. Retrying...")
                else:
                    print("Invalid header bytes. Retrying...")
            time.sleep(0.1)  # Delay to avoid overloading the serial connection
        
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
