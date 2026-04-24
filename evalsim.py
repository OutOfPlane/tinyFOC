import numpy as np
import matplotlib.pyplot as plt




# 1. Define the structure matching the C struct
# 'f4' = 32-bit float (float in C)
# 'u4' = 32-bit unsigned integer (unsigned long on Windows/MSVC)
bldc_dtype = np.dtype([
    ('angle', 'f4'),
    ('velocity', 'f4'),
    ('current_a', 'f4'),
    ('current_b', 'f4'),
    ('current_c', 'f4'),
    ('voltage_a', 'f4'),
    ('voltage_b', 'f4'),
    ('voltage_c', 'f4'),
    ('last_update', 'u4')
])

sens_dtype = np.dtype([
    ('last_update', 'u4'),
    ('val0', 'f4'),
    ('val1', 'f4'),
    ('val2', 'f4'),
    ('val3', 'f4'),
    ('val4', 'f4')
])

# 2. Load the entire file into a structured array
data = np.fromfile("motor_data.bin", dtype=bldc_dtype)
sns_data = np.fromfile("sensor_data.bin", dtype=sens_dtype)
t = data['last_update'] / 1e6  # Convert microseconds to seconds
angle_deg = np.unwrap(data['angle']) * 180 / np.pi  # Convert radians to degrees
sns_angle = np.unwrap(sns_data['val0']) * 180 / np.pi  # Convert radians to degrees
# 3. Plot the data
plt.figure(figsize=(10, 6))
plt.plot(t, angle_deg, label='Motor Angle (deg)')
plt.plot(t, sns_angle, label='Sensor Angle (deg)')
plt.plot(t, data['voltage_a'], label='Phase A Voltage (V)')
plt.plot(t, data['voltage_b'], label='Phase B Voltage (V)')
plt.plot(t, data['voltage_c'], label='Phase C Voltage (V)')
plt.plot(t, data['velocity'], label='Velocity (rad/s)')
plt.ylim(-100, 100)  # Adjust y-axis limits for better visibility

plt.xlabel('Time (seconds)')
plt.ylabel('Value')
plt.title('tinyFOC Simulation Data')
plt.legend()
plt.grid(True)
plt.show()