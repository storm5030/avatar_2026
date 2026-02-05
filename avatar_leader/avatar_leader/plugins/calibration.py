import math
import numpy as np

class Calibration():
    def __init__(self):
        self.leader_L1 = -0.25
        self.leader_L2 = -0.30 
        self.follower_L1 = -0.15
        self.follower_L2 = -0.20
        self.leader_offset = 0.20
        self.follwer_offset = 0.15

    def calibrate(self, leader_angles):
        # Step 1: Forward kinematics for leader to get end effector position
        leader_position = self.forward_kinematics_leader(leader_angles)
        
        # Step 2: Scale leader position to follower criteria
        follower_target_position = self.scaling_to_follower(leader_position)
        
        # Step 3: Inverse kinematics for follower to get joint angles
        initial_follower_angles = leader_angles  # Initialize with leader angles
        follower_angles = self.inverse_kinematics_follower(follower_target_position, initial_follower_angles)
        
        return follower_angles

    # Fowarad kinematics for leader (4 angles -> leader end effector position)
    def forward_kinematics_leader(self, angles):
        shoulder_pitch = angles[0]
        shoulder_roll = angles[1]
        elbow_yaw = angles[2]
        elbow_pitch = angles[3]
        # Calculate position of the end effector based on the angles and link lengths
        end_effector_position = np.array([0, self.leader_offset, 0]).T 
        end_effector_position += np.array(self.rotation_matrix_pitch(shoulder_pitch)) @ np.array(self.rotation_matrix_roll(shoulder_roll)) @ (np.array([0, 0, self.leader_L1]).T) 
        end_effector_position += np.array(self.rotation_matrix_pitch(shoulder_pitch)) @ np.array(self.rotation_matrix_roll(shoulder_roll)) @ np.array(self.rotation_matrix_yaw(elbow_yaw)) @ self.rotation_matrix_pitch(elbow_pitch) @ (np.array([0, 0, self.leader_L2]).T)
        return np.array(end_effector_position).T

    
    # Sclaling to followr arm criteria leader/follower arm ratio (leader end effectorposition -> follower end effector position)
    def scaling_to_follower(self, leader_position):
        scale_factor = (self.follower_L1 + self.follower_L2) / (self.leader_L1 + self.leader_L2)
        follower_position = leader_position * scale_factor
        return follower_position
    
    # Forward kinematics for follower (4 angles -> follower end effector position)
    def forward_kinematics_follower(self, angles):
        shoulder_pitch = angles[0]
        shoulder_roll = angles[1]
        elbow_yaw = angles[2]
        elbow_pitch = angles[3]
        # Calculate position of the end effector based on the angles and link lengths
        # Match the leader's structure: pitch first, then roll
        end_effector_position = np.array([0, self.follwer_offset, 0])
        end_effector_position += np.array(self.rotation_matrix_pitch(shoulder_pitch)) @ np.array(self.rotation_matrix_roll(shoulder_roll)) @ np.array([0, 0, self.follower_L1])
        end_effector_position += np.array(self.rotation_matrix_pitch(shoulder_pitch)) @ np.array(self.rotation_matrix_roll(shoulder_roll)) @ np.array(self.rotation_matrix_yaw(elbow_yaw)) @ np.array(self.rotation_matrix_pitch(elbow_pitch)) @ np.array([0, 0, self.follower_L2])
        return end_effector_position
    
    # Get Jacobian matrix for follower arm
    def get_jacobian_follower(self, angles):
        """Calculates the 3x4 Jacobian matrix for follower arm."""
        # Ensure angles is a numpy array
        angles = np.array(angles, dtype=float)
        
        # Compute partial derivatives of end effector position w.r.t. each joint angle
        # Using numerical differentiation for simplicity
        epsilon = 1e-6
        J = np.zeros((3, 4))
        
        current_pose = self.forward_kinematics_follower(angles)
        
        for i in range(4):
            angles_perturbed = angles.copy()
            angles_perturbed[i] += epsilon
            pose_perturbed = self.forward_kinematics_follower(angles_perturbed)
            J[:, i] = (pose_perturbed - current_pose) / epsilon
        
        return J
    
    # Inverse Kinematics for followr (initialized with leader angles) (follower end effector position -> 4 follower angles)
    def inverse_kinematics_follower(self, follower_position, initial_angles):
        
        theta = np.array(initial_angles, dtype=float)
        follower_position = np.array(follower_position, dtype=float)
        tol = 1e-6
        max_iter = 1000
        damping = 0.1  # Damping factor for stability
        
        for i in range(max_iter):
            current_pose = self.forward_kinematics_follower(theta)
            error = follower_position - current_pose
            
            if np.linalg.norm(error) < tol:
                print(f"IK Converged in {i} iterations. Final error: {np.linalg.norm(error):.2e}")
                return theta
            
            J = self.get_jacobian_follower(theta)
            # Damped least squares (Levenberg-Marquardt style) for better stability
            JTJ = J.T @ J
            damping_matrix = damping * np.eye(4)
            try:
                delta_theta = np.linalg.solve(JTJ + damping_matrix, J.T @ error)
                theta = theta + delta_theta
            except np.linalg.LinAlgError:
                print("Singular matrix, using pseudoinverse")
                try:
                    delta_theta = np.linalg.pinv(J) @ error
                    theta = theta + 0.5 * delta_theta  # Smaller step
                except:
                    print("Cannot invert Jacobian")
                    break
        
        print(f"IK did not fully converge after {max_iter} iterations. Final error norm: {np.linalg.norm(error):.2e}")
        return theta

    def rotation_matrix_roll(self, roll):
        return [
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ]
    
    def rotation_matrix_pitch(self, pitch):
        return [
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ]
    
    def rotation_matrix_yaw(self, yaw):
        return [
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ]