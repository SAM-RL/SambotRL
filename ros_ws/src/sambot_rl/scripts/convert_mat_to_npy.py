# @brief This is a helper script to convert .mat files of the original field into .npy files that
# can be loaded into C++ scripts. This saved file will be used for initializing the spatial-temporal
# varying fields for RL tasks.

import numpy as np
from scipy import io

def save_mat_as_npy(mat_file_path, npy_file_path):
    loaded_mat = io.loadmat(mat_file_path)
    u = loaded_mat.get("u")
    np.save(npy_file_path, u)

    # The following prints are to just make sure that the file loaded into C++ code matches with the
    # values here. Here, the 0th index refers to column, and 1th index refers to the row. Therefore,
    # the matching indices should be (row * 100 + col). That is, [23, 30] should match 3023.
    print(u[0, 0])
    print(u[40, 1])
    print(u[70, 5])
    print(u[0, 20])
    print(u[23, 30])
    print(u[5, 45])
    print(u[60, 55])
    print(u[99, 99])

if __name__ == "__main__":
    mat_file_path = "../data/initial_field/u.mat"
    npy_file_path = "../data/initial_field/u_npy.npy"

    save_mat_as_npy(mat_file_path, npy_file_path)
