from typing import Tuple

import numpy as np



def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")
    
    h = shape[1] // 2 
    
    res[0: shape[0], h : ] = -1
    res[0: shape[0], 0: h + 50 ] = 1
    
    res[0: 150, :] = 0
    
    res[150: 300, 0: 100 ] = -1
    res[150: 200, 100: 200 ] = -1


    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")
    
    h = shape[1] // 2
    
    res[0: shape[0], 0: h ] = -1
    res[0: shape[0], h :  ] = 1
    
    res[0: 150, :] = 0

    
    res[150: 200, shape[1] - 200: shape[1] - 100 ] = -1
    res[150: 300, shape[1] - 100: shape[1] ] = -1

    return res


