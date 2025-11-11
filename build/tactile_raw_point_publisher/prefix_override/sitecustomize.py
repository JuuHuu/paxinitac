import sys
if sys.prefix == '/home/juu/miniconda3/envs/paxinitac':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juu/Documents/paxinitac/install/tactile_raw_point_publisher'
