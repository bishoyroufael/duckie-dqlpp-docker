# https://stackoverflow.com/questions/16780014/import-file-from-parent-directory
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import cfg
import numpy as np

f = np.memmap('raw_d_mm', dtype=np.float16, mode='r+', shape=(cfg.depth_dim, cfg.depth_dim))

while True:
	print(f)
