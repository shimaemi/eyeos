from tf_luna import TFLuna
import time

def test_tf_luna():
    sensor = TFLuna('/dev/serial0', 115200)

    try:
        # Test setting baudrate
        print("Testing set_baudrate...")
        sensor.set_baudrate(115200)

        # Test reading and printing distance in various units
        print("Testing print_distance...")
        sensor.print_distance('cm')
        sensor.print_distance('inch')
        sensor.print_distance('ft')
        sensor.print_distance('m')
        sensor.print_distance('mm')

        # Test reading and printing temperature in various units
        print("Testing print_temperature...")
        sensor.print_temperature('C')
        sensor.print_temperature('F')
        sensor.print_temperature('K')

        # Test reading and printing signal strength
        print("Testing print_strength...")
        sensor.print_strength()

    finally:
        sensor.close()
        print("Sensor connection closed.")

if __name__ == "__main__":
    test_tf_luna()