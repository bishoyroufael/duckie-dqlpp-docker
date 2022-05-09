import numpy as np
import mmap

# Numpy memmap
f = np.memmap('raw_d_mm', dtype=np.float16, mode='w+', shape=(224,224))

while True:
    x = np.zeros((224,224), dtype=np.float16)
    x[:] = 0.5
    f[:] = x[:]