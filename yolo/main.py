import rospy
# ROS Image message
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2, sys
import torch
import numpy as np
from copy import deepcopy

THETA = 1.4 # rad

# global
humans = np.array([])
imgw = 640
xypub = None

# Instantiate CvBridge
bridge = CvBridge()

model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

def image_callback(msg):
	global humans, imgw

	if 1:
		# Convert your ROS Image message to OpenCV2
		frame = bridge.imgmsg_to_cv2(msg)
		imgw = frame.shape[1]
		# Save your OpenCV2 image as a jpeg
		results = model(frame)
		person = results.pandas().xyxy["name"=="person"].to_numpy()
		person = person[person[:,-2]==0,:]
		if person.shape[0] != 0:
			cx = 0.5*(person[:,0]+person[:,2])
			cy = 0.5*(person[:,1]+person[:,3])
			cxy = np.stack([cx,cy])
			humans = deepcopy(cx)
			try:
				for i in range(person.shape[0]):
					frame = cv2.circle(frame, (int(cxy[0,i]),int(cxy[1,i])), 2, (255,0,0), 10)
			except:
				pass
		else:
			humans = np.array([])




		cv2.imshow('frame', frame)
		if cv2.waitKey(1) == ord('q'):
			sys.exit()
	else:
		pass

def lidar_callback(msg):
	global humans, imgw, THETA
	global xypub
	
	dx = humans-(imgw//2)
	alpha = -dx*THETA/(imgw)
	alpha[alpha < 0] += 2*np.pi
	lidx = alpha/msg.angle_increment
	lidx = lidx.astype(int)
	l = (np.array(msg.ranges)[lidx]).astype(np.float32)
	alpha = alpha.astype(np.float32)
	x = l*np.cos(alpha)
	y = l*np.sin(alpha)


	data_to_send = Float32MultiArray()
	data_to_send.data = np.concatenate([x,y])
	xypub.publish(data_to_send)


	

def main():
	global xypub 

	rospy.init_node('image_listener')
	# Define your image topic
	xypub = rospy.Publisher('/humanxy', Float32MultiArray, queue_size=1)

	# Set up your subscriber and define its callback
	rospy.Subscriber("/VideoRaw", Image, image_callback)
	rospy.Subscriber('/scan', LaserScan, lidar_callback)
	# Spin until ctrl + c
	rospy.spin()

if __name__ == '__main__':
	main()