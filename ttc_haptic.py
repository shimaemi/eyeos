import time
from tf_luna import TFLuna

def main():
    # Initialize the TFLuna sensor
    sensor = TFLuna(port='/dev/ttyAMA0', baudrate=115200, pwm_pin=18)
    
    try:
        # Test reading distance and adjusting vibration intensity
        print("Starting TTC and vibration motor test...")
        sensor.print_ttc_velocity()
        
    except KeyboardInterrupt:
        print("Test interrupted by user.")
    finally:
        # Close the sensor and GPIO resources
        sensor.close()
        print("Sensor and GPIO resources closed.")

if __name__ == "__main__":
    main()