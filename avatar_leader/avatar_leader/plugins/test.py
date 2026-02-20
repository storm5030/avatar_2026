from calibration import Calibration
from math import pi

angles = [-pi/2, -pi/4, -pi/6, -pi/3]
print("Leader Angles:", angles)

calibrator = Calibration()

leader_position = calibrator.forward_kinematics_leader(angles, side='right')
print("Leader Right End Effector Position:", leader_position)

scaled_position = calibrator.scaling_to_follower(leader_position)
print("Scaled Right Follower Target Position:", scaled_position)

follower_angles = calibrator.calibrate(angles, side='right')
print("Calibrated Right Follower Angles:", follower_angles)

inverse_position = calibrator.forward_kinematics_follower(follower_angles, side='right')
print("Right Follower End Effector Position from Inverse Kinematics:", inverse_position)

leader_position_left = calibrator.forward_kinematics_leader(angles, side='left')
print("Leader Left End Effector Position:", leader_position_left)

scaled_position_left = calibrator.scaling_to_follower(leader_position_left)
print("Scaled Left Follower Target Position:", scaled_position_left)

follower_angles_left = calibrator.calibrate(angles, side='left')
print("Calibrated Left Follower Angles:", follower_angles_left)

inverse_position_left = calibrator.forward_kinematics_follower(follower_angles_left, side='left')
print("Left Follower End Effector Position from Inverse Kinematics:", inverse_position_left)

