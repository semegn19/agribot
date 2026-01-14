import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/semegn/agribot_ws/install/agribot_nav2_bringup'
