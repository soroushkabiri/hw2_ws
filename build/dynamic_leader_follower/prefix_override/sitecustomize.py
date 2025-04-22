import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/soroushkabiri92/hw2_ws/install/dynamic_leader_follower'
