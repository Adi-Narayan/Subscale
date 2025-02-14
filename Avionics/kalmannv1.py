import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Simple Kalman filter implementation in Python
class KalmanFilter:
    def __init__(self, Q=0.001, R=0.1, P=1):
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.P = P  # Estimate covariance
        self.K = 0  # Kalman gain
        self.X = 0  # Estimated state
    
    def filter(self, Z):
        # Prediction update
        self.P = self.P + self.Q
        
        # Measurement update
        self.K = self.P / (self.P + self.R)  # Kalman gain
        self.X = self.X + self.K * (Z - self.X)  # State estimate
        self.P = (1 - self.K) * self.P  # Update estimate covariance
        
        return self.X

data = pd.read_csv(r'D:\Anushka\AB MIT Manipal\thrustMIT\Avionics\No_Hopes_L.csv')

kf = KalmanFilter(Q=0.001, R=0.1)
filtered_data = []

for value in data['pressure']:
    filtered_data.append(kf.filter(value))

filtered_data = np.array(filtered_data)
raw_data = np.array(data['pressure'])

plt.figure(figsize=(10, 6))
plt.plot(raw_data, label='Raw Data', color='red', alpha=0.7)
plt.plot(filtered_data, label='Filtered Data (Kalman)', color='blue', alpha=0.7)
plt.xlabel('Time (samples)')
plt.ylabel('Sensor Readings')
plt.title('Raw vs Filtered Sensor Data')
plt.legend()
plt.grid(True)
plt.show()
