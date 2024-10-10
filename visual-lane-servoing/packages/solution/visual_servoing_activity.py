from typing import Tuple

import numpy as np
import cv2





def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """
    
    res = np.zeros(shape = shape, dtype="float32")
    
    rows, cols  = res.shape

    x = np.arange(cols)
    y = np.arange(rows)
    
    
 
    Ax, Ay = (120, 480)
    Bx, By = (300, 130)#100 --50th place
    k = (By - Ay) / (Bx - Ax)
    b = By - k * Bx
    first_line = k * x + b
    #TODO уменьши 185 до 175 и пробовать рисовать обычный треугольник 
    #TODO сдвинуть ближе к центру
    Ax, Ay = (270, 480)
    Bx, By = (300, 130)#100 --50th place
    k = (By - Ay) / (Bx - Ax)
    b = By - k * Bx
    second_line = k * x + b    

    #mask = (y[:, np.newaxis] >= k * x + b)  & (x >= Ax) & (x <= Bx)
    mask = (y[:, np.newaxis] >= first_line)  & (y[:, np.newaxis] <= second_line) 

    res[mask] = -1
    res[~mask] = 0
    
    return res


def get_steer_matrix_left_lane_markings2(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """
    
    res = np.zeros(shape = shape, dtype="float32")
    
    rows, cols  = res.shape

    x = np.arange(cols)
    y = np.arange(rows)
    
    
 
    Ax, Ay = (150, 480)
    Bx, By = (250, 270)#100 --50th place
    k = (By - Ay) / (Bx - Ax)
    b = By - k * Bx
    first_line = k * x + b
    #TODO уменьши 185 до 175 и пробовать рисовать обычный треугольник 
    #TODO сдвинуть ближе к центру
    Ax, Ay = (350, 480)
    Bx, By = (250, 270)#100 --50th place
    k = (By - Ay) / (Bx - Ax)
    b = By - k * Bx
    second_line = k * x + b    

    #mask = (y[:, np.newaxis] >= k * x + b)  & (x >= Ax) & (x <= Bx)
    mask = (y[:, np.newaxis] >= first_line)  & (y[:, np.newaxis] >= second_line) 

    res[mask] = -1
    res[~mask] = 0
    
    return res


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
                             using the masked right lane markings (numpy.ndarray)
    """
    
    res = np.zeros(shape = shape, dtype="float32")
    
    rows, cols  = res.shape
    x = np.arange(cols)
    y = np.arange(rows)
    
    #TODO сделать 210
    Ax, Ay = (310, 200)#100 --50th place, 250 -- 37
    Bx, By = (540 , 480)
    k = (By - Ay) / (Bx - Ax)
    b = By - k * Bx
    first_line = k * x + b 
 
    Ax, Ay = (310, 200)#100 --50th place, 250 -- 37
    Bx, By = (420 , 480)
    k = (By - Ay) / (Bx - Ax)
    b = By - k * Bx
    second_line = k * x + b

    #mask = (y[:, np.newaxis] >= k * x + b) & (x >= Ax) & (x <= Bx)
   
    mask = (y[:, np.newaxis] >= first_line)  & (y[:, np.newaxis] <= second_line) 
    
    res[mask] = 1
    res[~mask] = 0
    
    return res



def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        mask_left_edge:   Masked image for the dashed-yellow line (numpy.ndarray)
        mask_right_edge:  Masked image for the solid-white line (numpy.ndarray)
    """
    h, w, _ = image.shape
    
    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        

    # Smooth the image using a Gaussian kernel
    sigma = 3
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)

    
    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)

    threshold = 63 # np.random.rand(1,1) 
    mask_mag = (Gmag > threshold)

    white_lower_hsv = np.array([0, 0,168])         
    white_upper_hsv = np.array([172,111,255])  
    yellow_lower_hsv = np.array([22, 93, 0])      
    yellow_upper_hsv = np.array([45, 255, 255]) 

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv) 

    # In the left-half image, we are interested in the right-half of the dashed yellow line, which corresponds to negative x- and y-derivatives
    # In the right-half image, we are interested in the left-half of the solid white line, which correspons to a positive x-derivative and a negative y-derivative
    # Generate a mask that identifies pixels based on the sign of their x-derivative
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    
    mask_left_edge =   mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow #* get_steer_matrix_left_lane_markings((h,w))
    mask_right_edge =  mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white #* get_steer_matrix_right_lane_markings((h,w))
   # print(img_gaussian_filter.shape)
    #print( mask_left_edge[np.where(mask_left_edge != 0)])
    return mask_left_edge, mask_right_edge
