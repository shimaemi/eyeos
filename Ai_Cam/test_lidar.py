# Example usage of the updated TFLuna class
lidar_sensor = TFLuna(port='/dev/ttyserial0')  # Replace with your actual serial port

# Read and convert distance
distance = lidar_sensor.read_distance()
if distance is not None:
    # Convert distance to meters
    distance_in_meters = lidar_sensor.convert_distance(distance, 'm')
    print(f"Distance: {distance_in_meters:.2f} meters")

# Close the sensor connection when done
lidar_sensor.close()
