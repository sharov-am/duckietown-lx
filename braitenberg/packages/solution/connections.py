from typing import Tuple

import numpy as np



def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")
    
    #h = shape[1] // 2 
    
    #res[0: shape[0], h : ] = -1
    #res[0: shape[0], 0: h ] = 1
    
    #res[0: 150, :] = 0
    
    #res[150: 300, 0: 100 ] = -1
    #res[150: 200, 100: 200 ] = -1

    # given: matrix, Axy, Bxy 
    rows, cols = res.shape
    x = np.arange(cols)
    y = np.arange(rows)
    
    By = 0
    Ay = 350
    Bx = shape[1]//2
    Ax = 0
    k = (By - Ay) / (Bx - Ax)
    b = By - k * Bx
    mask = (y[:, np.newaxis] >= k * x + b) & (x >= Ax) & (x <= Bx)

    res[mask] = 1
    res[~mask] = -1
    
    #zero_mask = (y[:, np.newaxis] <= k * x + b) & (x >= Ax) & (x <= Bx)
    #res[zero_mask]  = 0
    res[0: 150, 0: shape[1] // 2 ] = -1
    
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape = shape, dtype="float32")
    
    #h = shape[1] // 2
    
    #res[0: shape[0], 0: h ] = -1
    #res[0: shape[0], h :  ] = 1
    
    #res[0: 150, :] = 0

    
    #res[150: 200, shape[1] - 200: shape[1] - 100 ] = -1
    #res[150: 300, shape[1] - 100: shape[1] ] = -1

    rows, cols = res.shape
    x = np.arange(cols)
    y = np.arange(rows)
    
    By = 350
    Ay = 0
    Bx = shape[1]
    Ax = 300
    k = (By - Ay) / (Bx - Ax)
    b = By - k * Bx
    mask = (y[:, np.newaxis] >= k * x + b) & (x >= Ax) & (x <= Bx)

    res[mask] = 1
    res[~mask] = -1
    
    #zero_mask = (y[:, np.newaxis] <= k * x + b) & (x >= Ax) & (x <= Bx)
    #res[zero_mask]  = 0
    
    res[0: 150, (shape[0] // 2) :] = -1
    
    return res


