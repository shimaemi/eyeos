from tf_luna import TFLuna
import time

def test_tf_luna():
    sensor = TFLuna('/dev/serial0', 115200)

    # Test setting sample rate
    print("Setting sample rate")
    sensor.set_sample(50)

    # Test get sample rate
    print("Checking sample rate")
    sample = sensor.get_sample
    print("Sample rate = " + str(sample) + "hz")

    # Test get period
    print("Checking period")
    period = sensor.get_period
    print("Period = " + str(period) + "s")

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

    # Test time to collide
    print("Testing ttc and velocity reads")
    while True:
        sensor.print_velocity()     



if __name__ == "__main__":
    try:
        test_tf_luna()
    except KeyboardInterrupt:
        sensor.close()
        print("program interrupted by the user")