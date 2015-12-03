import signal
import sys
import freenect
import math
# import bluetooth
import numpy as np 
import cv2
from freenect import sync_get_depth as get_depth,DEPTH_MM 
from matplotlib import pyplot as plt

#command line arguments:
if len(sys.argv) is not 6:
	print 'require 5 command line arguments'
	print 'argv[1] = lower limit of depth slice'
	print 'argv[2] = upper limit of depth slice'
	print 'argv[3] = video name .avi'
	print 'argv[4] = lower limit of area'
	print 'argv[5] = upper limit of area'
	exit()


###########################################################
# Edit these parameters for configuring on field
###########################################################

# limits for depth slicing 
l1 = int(sys.argv[1])
l2 = int(sys.argv[2])

vid_name = sys.argv[3]

# lower and higher bound on area of object to be
# detected as shuttle
area_low = int(sys.argv[4])
area_high = int(sys.argv[5])

# this is time(in seconds) after which the timer 
# loop has to auto-terminate after first time 
# the shuttle is seen
timelimit = 3

# status of bluetooth
bton = False

# this is the height measured from top 
# actually it is inverted
height_required = 370

Dz = 750
Dx = 6700
Dy = 1500

###########################################################
###########################################################

global heightmask
mask1 = np.empty((height_required, 640)); mask1.fill(True)
mask2 = np.zeros((480 - height_required, 640))
heightmask = np.concatenate((mask1, mask2), axis=0)

def getBinaryImage(l1, l2):
	(d,_) = get_depth(format = DEPTH_MM)
	m1 = d < l2
	m2 = d > l1
	m = np.logical_and(m1, m2)
	m = np.logical_and(m,heightmask)
	m = m.astype(np.uint8)
	m = m*255
	return (m,d)

def handler(signum, frame):
	video_writer.release()
	freenect.sync_stop()
	exit()

def convert(x, y, z_real):
	# conversion of pixel coordinates to real world
	f_x = 320/math.tan(math.pi/180*(43.5/2))
	f_y = 240/math.tan(math.pi/180*(57/2))
	# adjusting kinect orgin to centre from top left corner
	x_kcr = (320-x)*z_real/f_x
	y_kcr = (240-y)*z_real/f_y
	# adjusting coordinates with reference to mapping origin
	x_mor = x_kcr + Dx
	y_mor = y_kcr + Dy
	z_mor = z_real - Dz
	return (x_mor,y_mor,z_mor)
	
if __name__ == "__main__":
	# for Ctrl+C to work
	signal.signal(signal.SIGINT, handler)
	# video for debugging
	fourcc = cv2.cv.FOURCC(*'XVID')
	
	if bton is True:
		import bluetooth
		bd_addr= '10:14:07:10:38:12'
		port = 1
		socket=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		socket.connect((bd_addr,port))

	video_writer = cv2.VideoWriter(sys.argv[3], fourcc, 15	, (640, 480))
    # record video

	global contours, depth, d1, i, count, b, A 
	select = 0
	flag = False
	while flag is False:
		depth, d1 = getBinaryImage(l1, l2)
		depth1 = depth.copy()
		contours, _ = cv2.findContours(depth1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		l = len(contours)
		for i in range(0, l):
			area = cv2.contourArea(contours[i])
			if area > area_low and area < area_high:
				select = i
				flag = True
				break
		pass

	t0 = cv2.getTickCount()   
	M = cv2.moments(contours[i])
	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])
	z = d1[cy][cx].astype(np.float32)
	(X,Y,Z) = convert(cx,cy,z)
	Z = z 
	SumX = X 
	SumZ = Z 
	SumX2 = X*X 
	SumZ2 = Z*Z 
	SumXZ = X*Z 
	count = 1 

	while True:
		depth,d1 = getBinaryImage(l1, l2)
		depth1 = depth.copy()
		contours, _ = cv2.findContours(depth1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		l = len(contours)
		depth = np.dstack((depth, depth, depth))
		t1 = cv2.getTickCount()
		timediff = (t1-t0)/cv2.getTickFrequency()
		if timediff > timelimit:
			break
		for i in range(0,l):
			cnt = contours[i]
			area = cv2.contourArea(cnt)
			if area > area_low and area < area_high:
				print timediff
				print 'count: '+str(count)
				print 'Area: '+str(area)
				cv2.drawContours(depth, contours, i, (255, 0, 0), 3)
				M = cv2.moments(cnt)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				z = d1[cy][cx].astype(np.float32)
				(X,Y,Z) = convert(cx,cy,z)
				print 'Real Centroid Coordinates'
				print (X,Y,Z)

				count += 1
				SumX = SumX + X 
				SumZ = SumZ + Z 
				SumX2 = SumX2 + X*X 
				SumZ2 = SumZ2 + Z*Z 
				SumXZ = SumXZ + X*Z 
				Mx = SumX/count
				Mz = SumZ/count
				Sx = math.sqrt(SumX2/count - Mx*Mx)
				Sz = math.sqrt(SumZ2/count - Mz*Mz)
				r = (SumXZ/count - Mz*Mx)/(Sx*Sz)

				b = r * Sx/Sz
				A = Mz - b*Mx

				print '-------------------------'
				break

		video_writer.write(depth)

	# equation format y = bx + A
	print (b,A)
	if bton is True:
		socket.send(str(b)+'$-1$'+str(A)+'$$')

	#socket.send()

