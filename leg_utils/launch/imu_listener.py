import matplotlib.pyplot as plt
import re

def read_imu_data(filename):
    with open(filename, 'r') as file:
        data = file.read()
    linear_acceleration_values = re.findall(r'linear_acceleration:\s*\n\s*x:\s*(-?\d+\.\d+)\n\s*y:\s*(-?\d+\.\d+)\n\s*z:\s*(-?\d+\.\d+)', data)
    angular_velocity_values = re.findall(r'angular_velocity:\s*\n\s*x:\s*(-?\d+\.\d+)\n\s*y:\s*(-?\d+\.\d+)\n\s*z:\s*(-?\d+\.\d+)', data)
    time_values = re.findall(r'secs:\s*(\d+)\n\s*nsecs:\s*(\d+)', data)
    time_values = [float(sec) + float(nsec) / 1e9 for sec, nsec in time_values]
    linear_acceleration_values = [(float(x), float(y), float(z)) for x, y, z in linear_acceleration_values]
    angular_velocity_values = [(float(x), float(y), float(z)) for x, y, z in angular_velocity_values]
    return time_values, linear_acceleration_values, angular_velocity_values

# Чтение данных из файлов
time_values_1, linear_acceleration_values_1, angular_velocity_values_1 = read_imu_data('data_1.txt')
time_values_2, linear_acceleration_values_2, angular_velocity_values_2 = read_imu_data('data_5.txt')

# Построение графиков
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
# plt.plot(time_values_1, [acc[0] for acc in linear_acceleration_values_1], label='Linear Acceleration X (File 1)')
# plt.plot(time_values_1, [acc[1] for acc in linear_acceleration_values_1], label='Linear Acceleration Y (File 1)')
plt.plot(time_values_1, [acc[2] for acc in linear_acceleration_values_1], label='Linear Acceleration Z (File 1)')
# plt.plot(time_values_2, [acc[0] for acc in linear_acceleration_values_2], label='Linear Acceleration X (File 2)', linestyle='--')
# plt.plot(time_values_2, [acc[1] for acc in linear_acceleration_values_2], label='Linear Acceleration Y (File 2)', linestyle='--')
plt.plot(time_values_2, [acc[2] for acc in linear_acceleration_values_2], label='Linear Acceleration Z (File 2)', linestyle='--')
plt.xlabel('Time (secs)')
plt.ylabel('Linear Acceleration (m/s^2)')
plt.title('Linear Acceleration Comparison')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time_values_1, [ang[0] for ang in angular_velocity_values_1], label='Angular Velocity X (File 1)')
# plt.plot(time_values_1, [ang[1] for ang in angular_velocity_values_1], label='Angular Velocity Y (File 1)')
# plt.plot(time_values_1, [ang[2] for ang in angular_velocity_values_1], label='Angular Velocity Z (File 1)')
plt.plot(time_values_2, [ang[0] for ang in angular_velocity_values_2], label='Angular Velocity X (File 2)', linestyle='--')
# plt.plot(time_values_2, [ang[1] for ang in angular_velocity_values_2], label='Angular Velocity Y (File 2)', linestyle='--')
# plt.plot(time_values_2, [ang[2] for ang in angular_velocity_values_2], label='Angular Velocity Z (File 2)', linestyle='--')
plt.xlabel('Time (secs)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity Comparison')
plt.legend()

plt.tight_layout()
plt.show()
