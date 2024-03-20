import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

csv_data1 = """
0.21,379.00,403.00,
0.56,305.00,329.00,
0.93,251.00,262.00,
1.32,204.00,222.00,
1.71,177.00,187.00,
2.10,153.00,169.00,
2.49,135.00,145.00,
2.88,120.00,132.00,
3.27,108.00,115.00,
3.66,102.00,109.00,
4.05,90.00,101.00,
4.44,82.00,66.00,
4.83,73.00,82.00,
5.22,73.00,71.00,
"""
csv_data2 = """
0.22,383.00,405.00,
0.57,300.00,331.00,
0.95,250.00,264.00,
1.34,213.00,217.00,
1.73,171.00,192.00,
2.12,154.00,168.00,
2.51,137.00,147.00,
2.89,122.00,131.00,
3.28,106.00,117.00,
3.66,85.00,110.00,
4.05,87.00,102.00,
4.44,76.00,88.00,
4.83,80.00,80.00,
5.21,72.00,78.00,
"""

csv_data3 = """
0.21,388.00,411.00,
0.57,307.00,330.00,
0.95,250.00,260.00,
1.34,211.00,219.00,
1.73,179.00,187.00,
2.12,148.00,169.00,
2.51,133.00,147.00,
2.90,121.00,130.00,
3.28,110.00,116.00,
3.67,101.00,110.00,
4.06,90.00,102.00,
4.45,91.00,95.00,
4.84,64.00,84.00,
5.22,73.00,71.00,
"""

# Function to parse CSV data into numpy array
def parse_csv(csv_data):
    return np.array([
        [float(value) for value in row.split(',')[:-1]]
        for row in csv_data.strip().split('\n')
    ])

data1 = parse_csv(csv_data1)
data2 = parse_csv(csv_data2)
data3 = parse_csv(csv_data3)

# Compute the average of corresponding values from the three datasets
data_avg = (data1 + data2 + data3) / 3

# Extracting columns from the averaged data
inches = data_avg[:, 0]
left_sensor = data_avg[:, 1]
right_sensor = data_avg[:, 2]

# Performing linear regression on the log-transformed data
slope_left_log, intercept_left_log, _, _, _ = linregress(np.log(inches), np.log(left_sensor))
slope_right_log, intercept_right_log, _, _, _ = linregress(np.log(inches), np.log(right_sensor))

# Print coefficients
print(f"Left Sensor Coefficients:\nSlope: {slope_left_log}\nIntercept: {intercept_left_log}")
print(f"Right Sensor Coefficients:\nSlope: {slope_right_log}\nIntercept: {intercept_right_log}")

# Plotting
plt.figure(figsize=(10, 6))
plt.loglog(inches, left_sensor, 'o', label='Left Sensor', color='blue')
plt.loglog(inches, right_sensor, 'x', label='Right Sensor', color='red')
plt.xlabel('Inches (log scale)')
plt.ylabel('Sensor Readings (log scale)')
plt.title('Sensor Readings by Distance (Log-Log Scale)')
plt.legend()
plt.grid(True, which="both", ls="--")
plt.show()