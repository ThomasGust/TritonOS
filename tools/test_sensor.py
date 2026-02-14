import bluerobotics_navigator as navigator

print("Initializing navigator module.")
navigator.init()

print("Setting led on!")
navigator.set_led(navigator.UserLed.Led1, True)

print(f"Temperature: {navigator.read_temp()}")
print(f"Pressure: {navigator.read_pressure()}")
data = navigator.read_mag()
print(f"Magnetic field: X = {data.x}, Y = {data.y}, Z = {data.z}")
data = navigator.read_gyro()
print(f"Gyroscope: X = {data.x}, Y = {data.y}, Z = {data.z}")
data = navigator.read_accel()
print(f"Accelerometer: X = {data.x}, Y = {data.y}, Z = {data.z}")
