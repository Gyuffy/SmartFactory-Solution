import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ssafy/Desktop/SmartFactory-Solution/laptop1/install/pjt'
