import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

csv_data = """
0.58,315.00,334.00,
0.94,260.00,256.00,
1.30,222.00,228.00,
1.68,188.00,195.00,
2.06,165.00,166.00,
2.45,145.00,152.00,
2.84,129.00,146.00,
3.24,116.00,132.00,
3.62,111.00,121.00,
4.01,99.00,114.00,
4.40,90.00,102.00,
4.78,89.00,98.00,
5.17,78.00,85.00,
5.55,77.00,80.00,
"""

# Parse csv
data = np.array([
    [float(value) for value in row.split(',')[:-1]]  # Skip the last, empty entry
    for row in csv_data.strip().split('\n')
])

# Extracting columns
inches = data[:, 0]
left_sensor = data[:, 1]
right_sensor = data[:, 2]

# Performing linear regression on the log-transformed data
slope_left_log, intercept_left_log, _, _, _ = linregress(np.log(inches), np.log(left_sensor))
slope_right_log, intercept_right_log, _, _, _ = linregress(np.log(inches), np.log(right_sensor))

# Printing coefficients
print(f"Left Sensor Coefficients:\nSlope: {slope_left_log}\nIntercept: {intercept_left_log}")
print(f"Right Sensor Coefficients:\nSlope: {slope_right_log}\nIntercept: {intercept_right_log}")

# Optional: Plotting
plt.figure(figsize=(10, 6))
plt.loglog(inches, left_sensor, 'o', label='Left Sensor', color='blue')
plt.loglog(inches, right_sensor, 'x', label='Right Sensor', color='red')
plt.xlabel('Inches (log scale)')
plt.ylabel('Sensor Readings (log scale)')
plt.title('Sensor Readings by Distance (Log-Log Scale)')
plt.legend()
plt.grid(True, which="both", ls="--")
plt.show()
