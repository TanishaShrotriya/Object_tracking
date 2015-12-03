import numpy as np 
import cv2
from freenect import sync_get_depth as get_depth,DEPTH_MM 
from matplotlib import pyplot as plt

def getBinaryImage(l1, l2):
	(d,_) = get_depth(format = DEPTH_MM)
	m1 = d < l2
	m2 = d > l1
	m = np.logical_and(m1, m2)
	m = m.astype(np.uint8)
	m = m*255
	return (m,d)
