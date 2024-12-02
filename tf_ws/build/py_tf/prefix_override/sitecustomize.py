import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/interstellar/tf_demo/tf_ws/install/py_tf'
