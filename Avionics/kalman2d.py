import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Kalman filter variables
x = np.array([0, 0], dtype=float)  # State vector [altitude, velocity]
P = np.array([[1, 0], [0, 1]], dtype=float)  # Covariance matrix
Q = np.array([[0.01, 0], [0, 0.03]], dtype=float)  # Process noise covariance
R = 0.5  # Measurement noise covariance
dt = 0.01  # Time step
g = 9.81  # Gravity

def kalman_filter(measured_altitude, accel_y):
    global x, P
    
    # Prediction step
    accel_earth = accel_y + g  # Convert acceleration to earth frame
    x[0] += x[1] * dt + 0.5 * accel_earth * dt**2  # Altitude update
    x[1] += accel_earth * dt  # Velocity update
    
    # Predict covariance matrix
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q[0][0])
    P[0][1] += dt * (P[1][1] + Q[0][1])
    P[1][0] += dt * (P[1][1] + Q[1][0])
    P[1][1] += Q[1][1]
    
    # Measurement update
    y = measured_altitude - x[0]  # Innovation
    S = P[0][0] + R  # Measurement uncertainty
    K = np.array([P[0][0] / S, P[1][0] / S])  # Kalman gain
    
    # Update state
    x[0] += K[0] * y
    x[1] += K[1] * y
    
    # Update covariance
    P[0][0] -= K[0] * P[0][0]
    P[0][1] -= K[0] * P[0][1]
    P[1][0] -= K[1] * P[0][0]
    P[1][1] -= K[1] * P[0][1]
    
    return x[0], x[1]  # Return updated altitude and velocity

# Read CSV file
def process_csv():
    df = pd.read_csv(r'D:\Anushka\AB MIT Manipal\thrustMIT\Avionics\No_Hopes_L.csv')
    
    results = []
    for _, row in df.iterrows():
        altitude = 44330 * (1.0 - (row['pressure'] / 101325.0) ** 0.1903)  # Convert pressure to altitude
        accel_y = row['bno_y']  # Use y-axis acceleration
        
        filtered_altitude, filtered_velocity = kalman_filter(altitude, accel_y)
        results.append((row['time'], filtered_altitude, filtered_velocity))
    
    return pd.DataFrame(results, columns=['time', 'filtered_altitude', 'filtered_velocity'])

# Plot function
def plot_results(df):
    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(df['time'], df['filtered_altitude'], label='Filtered Altitude', color='b')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('Filtered Altitude over Time')
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(df['time'], df['filtered_velocity'], label='Filtered Velocity', color='r')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Filtered Velocity over Time')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

# Example usage:
# filtered_data = process_csv("data.csv")
# filtered_data.to_csv("filtered_output.csv", index=False)
# plot_results(filtered_data)
# print(filtered_data.head())