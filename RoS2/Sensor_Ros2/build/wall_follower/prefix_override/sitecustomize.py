import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/Intel_study/Intel7_Final_Project_5_Team/RoS2/Sensor_Ros2/install/wall_follower'
