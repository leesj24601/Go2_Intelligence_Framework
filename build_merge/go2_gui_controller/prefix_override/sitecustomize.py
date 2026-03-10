import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cvr/Desktop/sj/go2_intelligence_framework/install_merge'
