import numpy as np

def rot_tran_to_homo(rotation_matrix, tvec):
    extrinsic = np.append(rotation_matrix, tvec, axis=1)
    bottom_row = [[0,0,0,1]]
    extrinsic = np.append(extrinsic, bottom_row, axis=0)
    return extrinsic