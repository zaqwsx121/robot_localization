import numpy as np
import matplotlib.pyplot as plt

# Read data from file
data = np.loadtxt('odom_data.txt')

# Calculate the rate of change of yaw
yaw_data = data[:, 2]
yaw_rate_change = np.diff(yaw_data)

# Append a zero at the beginning to match the length of original data
yaw_rate_change = np.insert(yaw_rate_change, 0, 0)

average_yaw_change = np.mean(yaw_rate_change)

print("Average Yaw Change: ", average_yaw_change)

# Plot three angles in separate subplots
plt.figure(figsize=(10, 8))

plt.subplot(411)
plt.plot(data[:, 0], label='theta1')
plt.legend()

plt.subplot(412)
plt.plot(data[:, 1], label='theta2')
plt.legend()

plt.subplot(413)
plt.plot(data[:, 2], label='theta3')
plt.legend()

# Plotting the yaw rate change
plt.subplot(414)
plt.plot(yaw_rate_change, label='Yaw Rate Change')
plt.legend()

plt.show()