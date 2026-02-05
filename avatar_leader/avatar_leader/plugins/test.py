from calibration import Calibration
from math import pi

angles = [-pi/2, -pi/4, -pi/6, -pi/3]
print("Leader Angles:", angles)

calibrator = Calibration()

leader_position = calibrator.forward_kinematics_leader(angles)
print("Leader End Effector Position:", leader_position)

scaled_position = calibrator.scaling_to_follower(leader_position)
print("Scaled Follower Target Position:", scaled_position)

follower_angles = calibrator.calibrate(angles)
print("Calibrated Follower Angles:", follower_angles)