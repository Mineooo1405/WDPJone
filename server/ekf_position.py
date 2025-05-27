import numpy as np
import math

class OmniRobotEKF:
    """Extended Kalman Filter for omnidirectional robot positioning"""
    
    def __init__(self):
        # State vector [x, y, theta, v_x, v_y]
        self.x = np.zeros((5, 1))
        
        # State transition matrix
        self.F = np.eye(5)
        self.F[0, 3] = 0.1  # x += v_x * 0.1
        self.F[1, 4] = 0.1  # y += v_y * 0.1
        # State covariance matrix
        self.P = np.eye(5) * 100
        
        # Process noise covariance 
        self.Q = np.eye(5)
        self.Q[0, 0] = 0.05  # x position
        self.Q[1, 1] = 0.05  # y position
        self.Q[2, 2] = 0.005  # theta
        self.Q[3, 3] = 0.05   # v_x
        self.Q[4, 4] = 0.05   # v_y
        
        # Measurement noise covariance [theta, v_x, v_y]
        self.R = np.eye(3)
        self.R[0, 0] = 0.01  # theta measurement noise
        self.R[1, 1] = 0.5   # v_x measurement noise
        self.R[2, 2] = 0.5   # v_y measurement noise
        
        # Ma trận điều khiển (5 trạng thái, 2 điều khiển)
        # self.B = np.zeros((5, 2))  
        # self.B[0, 0] = 0.1  # vx ảnh hưởng trực tiếp đến x
        # self.B[1, 1] = 0.1  # vy ảnh hưởng trực tiếp đến y
        # Last update time
        self.last_time = None
        
        # Flag indicating if filter has been initialized
        self.initialized = False
    
    def initialize(self, x, y, theta):
        """Initialize filter with position data"""
        self.x = np.array([[x], [y], [theta], [0.0], [0.0]])
        self.initialized = True
        
    def predict(self, u):
        """Predict step - update state based on model"""
        if not self.initialized:
            return
        

        # Predict state
        # u = np.array(u).reshape(2, 1)  # Đảm bảo u là vector cột
        self.x = self.F @ self.x
        
        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, z_theta, z_vx, z_vy):
        """Update with measurement z = [theta, v_x, v_y]"""
        if not self.initialized:
            return
            
        # Measurement is [theta, v_x, v_y]
        z = np.array([[z_theta], [z_vx], [z_vy]])
        
        # Measurement matrix - relates state to measurement
        H = np.zeros((3, 5))
        H[0, 2] = 1.0  # theta
        H[1, 3] = 1.0  # v_x
        H[2, 4] = 1.0  # v_y
        
        # Innovation (measurement residual)
        y = z - H @ self.x
        
        # Normalize theta error to [-pi, pi]
        y[0, 0] = math.atan2(math.sin(y[0, 0]), math.cos(y[0, 0]))
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state estimate
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(self.x.shape[0])
        self.P = (I - K @ H) @ self.P
        
    def get_state(self):
        """Return current state estimate"""
        if not self.initialized:
            return None
        
        return {
            'x': float(self.x[0, 0]),
            'y': float(self.x[1, 0]),
            'theta': float(self.x[2, 0]),
            'v_x': float(self.x[3, 0]),
            'v_y': float(self.x[4, 0])
        }

# Singleton instance
ekf = OmniRobotEKF()