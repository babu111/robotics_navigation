import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gixstudent/Desktop/Final lab/robotics_navigation/install/turtlebot3_teleop'
