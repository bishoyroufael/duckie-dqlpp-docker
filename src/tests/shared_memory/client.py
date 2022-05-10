
import numpy as np
# Numpy Example
f = np.memmap('raw_d_mm', dtype=cfg.dtype, mode='r+', shape=(224,224))

while True:
    print(f.var())