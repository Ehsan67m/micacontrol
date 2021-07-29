#!/usr/bin/env python

import rospy
import cv2
import os
import platform
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib
import matplotlib.pyplot as plt
from micasense_ros.msg import Multispectral
from matplotlib.gridspec import GridSpec
import time
from datetime import datetime
import warnings
warnings.filterwarnings("ignore")

plt.rcParams['toolbar'] = 'None'
BAND_NAME = ('Blue','Green','Red','NIR','RedEdge', 'RGB', 'Detected')
bridge=CvBridge()
bands=Multispectral()
process_done=False


def figure_init():
	global fig
	if not rospy.is_shutdown():
		fig=plt.figure(num="Bands viewer", figsize=(15,6))
		ax=[]
		gs = GridSpec(3, 6, wspace=0.01, hspace=0.0, top=0.99, bottom=0.005, left=0.005, right=0.99)
		ax.append(fig.add_subplot(gs[0:2, 0]))
		ax.append(fig.add_subplot(gs[0:2, 1]))
		ax.append(fig.add_subplot(gs[0:2, 2]))
		ax.append(fig.add_subplot(gs[2:4, 0]))
		ax.append(fig.add_subplot(gs[2:4, 1]))
		ax.append(fig.add_subplot(gs[2:4, 2]))
		ax.append(fig.add_subplot(gs[0:4, 3:7]))
		for i, axs in enumerate(fig.axes):
			if i<3:
				axs.text(0,0,BAND_NAME[i], rotation="horizontal", va="bottom", ha="left")
			elif i<6:
				axs.text(0,0,BAND_NAME[i], rotation="horizontal", va="bottom", ha="left")
			else:
				axs.text(0,0,BAND_NAME[i],rotation="horizontal", va="bottom", ha="left")
			axs.axis('off')

def receivecallback(bands):
	global process_done
	if process_done==False:
		callback(bands)
	else:
		return 

def callback(bands):
	try:
		global lastShowTime
		global platformRelease
		global process_done
		global txt
		if not rospy.is_shutdown():
			for imageIndex in range(5):
				print("........................")
				rospy.loginfo(bands.raw_bands[imageIndex].header)
				rospy.loginfo(bands.capture_id)
				rospy.loginfo(bands.file_name)
				rospy.loginfo(bands.time_string)
			print("===========================")
			period_time=((datetime.now()-lastShowTime).total_seconds())
			if ( period_time > 0) or ((platformRelease >4) and (period_time > 1.5)):
				process_done=True

				lastShowTime=datetime.now()
				### lowspeed loop!
				width = int(bands.raw_bands[0].width * 20 / 100)
				height = int(bands.raw_bands[0].height * 20 / 100)
				dim = (width, height)
				for i, axs in enumerate(fig.axes):
					if i != 5 and i != 6:
						imcv=bridge.imgmsg_to_cv2(bands.raw_bands[i],"16UC1")
						#im_rgb = cv2.cvtColor(imcv, cv2.COLOR_GRAY2RGB)
						new_image = cv2.resize(imcv, dim)
						axs.imshow(new_image)
						#axs.imshow(im_rgb)
					else:
						imcv=bridge.imgmsg_to_cv2(bands.raw_bands[i],"8UC3")
						if i==5:
							new_image = cv2.resize(imcv, dim)
							im_rgb = cv2.cvtColor(new_image, cv2.COLOR_BGR2RGB)
						else:
							im_rgb = cv2.cvtColor(imcv, cv2.COLOR_BGR2RGB)
						axs.imshow(im_rgb)

				###
				#print("===========================")
				plt.suptitle("Bands sequence: %d             " %(bands.raw_bands[0].header.seq),va="top", ha="right")
				#plt.text(0,0,"                                  \n                                 \n                                 \n",ha="right", va="top")
				plt.text(0,0,"File Name: "+bands.file_name+"\nID: "+bands.capture_id+"\nTime: " + bands.time_string ,ha="right", va="top", backgroundcolor='0.75')
				plt.draw()
				process_done=False

	except:
		rospy.signal_shutdown("You pressed Ctrl + C or closed the figure window.")

if __name__ == '__main__':
	platformRelease=int(platform.release()[0])
	print("Platform release first number: %d" % platformRelease)
	print("Close figure window to exit.")
	print('ROS node "mipviewer" subscribes to "detect_viewer" topic to receive "micasense_ros/Multispectral" messages and show multispectral bands and image processing results.')
	try:
		lastShowTime=datetime.now()
		rospy.init_node('mipviewer',anonymous=False,disable_signals=False)
		rospy.Subscriber("detect_viewer",Multispectral,receivecallback)
		init_flag=False
		while True:
			if rospy.is_shutdown():
				print("\nYou pressed Ctrl + C (ROS node stopped), close the figure window to exit.")
				break
			if init_flag==False:
				figure_init()
				init_flag=True
				plt.show()
			elif not plt.fignum_exists(fig.number):
				rospy.signal_shutdown("You pressed Ctrl + C or closed the figure window.")
				break
		rospy.signal_shutdown("You pressed Ctrl + C or closed the figure window.")
		print("Shutting down")
	except:
		raise Exception("Error! Shutting down")
