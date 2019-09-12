import h5py
import numpy as np
filepath = 'thomas_fb.mat'
arrays = {}
f = h5py.File(filepath)
for k, v in f.items():
    arrays[k] = np.array(v)

