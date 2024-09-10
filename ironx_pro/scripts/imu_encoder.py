import numpy as np

# Initial state (position and velocity)
x = np.array([[0], [0]])  # Initial position and velocity

# State transition matrix
dt = 1  # Time step
F = np.array([[1, dt], [0, 1]])

# Process covariance matrix
Q = np.array([[1, 0], [0, 1]])

# Measurement matrix
H = np.array([[1, 0]])

# Measurement covariance matrix
R = np.array([[1]])

# Initial estimate covariance
P = np.array([[1, 0], [0, 1]])

def predict(x, P, F, Q):
    x = np.dot(F, x)
    P = np.dot(np.dot(F, P), F.T) + Q
    return x, P

def update(x, P, z, H, R):
    y = z - np.dot(H, x)
    S = np.dot(np.dot(H, P), H.T) + R
    K = np.dot(np.dot(P, H.T), np.linalg.inv(S))
    x = x + np.dot(K, y)
    P = P - np.dot(np.dot(K, H), P)
    return x, P

# Simulated measurements
imu_data = [0.1, 0.2, 0.15, 0.3]  # Accelerometer data (e.g., velocity increments)
encoder_data = [0.1, 0.3, 0.45, 0.75]  # Encoder data (e.g., position)

for i in range(len(imu_data)):
    # Predict
    x, P = predict(x, P, F, Q)
    
    # Update with encoder data
    z = np.array([[encoder_data[i]]])
    x, P = update(x, P, z, H, R)
    
    print(f"State estimate after step {i+1}: {x.flatten()}")

