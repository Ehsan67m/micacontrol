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
import argparse
import warnings
warnings.filterwarnings("ignore")

parser = argparse.ArgumentParser()

parser.add_argument("-band", "--band", dest = "band_flag", help="Which band to view: 1(=band 1),..., 5(=band 5), 0(=all). (default: 0)",type=int, default=0, choices=range(6))

plt.rcParams['toolbar'] = 'None'
BAND_NAME = ('Blue','Green','Red','NIR','RedEdge')
bridge=CvBridge()
bands=Multispectral()
process_done=False

args = parser.parse_args()

def figure_init(band_flag):
	global fig
	if not rospy.is_shutdown():
		if band_flag == 0:
			fig=plt.figure(num="Bands viewer", figsize=(16,3.3))
			ax=[]
			gs = GridSpec(1, 5, wspace=0.01, hspace=0.0, top=0.99, bottom=0.005, left=0.005, right=0.99)
			ax.append(fig.add_subplot(gs[0, 0]))
			ax.append(fig.add_subplot(gs[0, 1]))
			ax.append(fig.add_subplot(gs[0, 2]))
			ax.append(fig.add_subplot(gs[0, 3]))
			ax.append(fig.add_subplot(gs[0, 4]))
			for i, axs in enumerate(fig.axes):
				axs.text(0,0,BAND_NAME[i], rotation="horizontal", va="bottom", ha="left")
				axs.axis('off')
		else:
			fig=plt.figure(num="Band "+BAND_NAME[band_flag-1]+" viewer", figsize=(12,6))
			ax=[]
			gs = GridSpec(1, 1, wspace=0.01, hspace=0.0, top=0.99, bottom=0.005, left=0.005, right=0.99)
			ax.append(fig.add_subplot(gs[0, 0]))
			for i, axs in enumerate(fig.axes):
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
		global band_flag
		if not rospy.is_shutdown():
			for imageIndex in range(5):
				print("........................")
				rospy.loginfo(bands.raw_bands[imageIndex].header)
				rospy.loginfo(bands.capture_id)
				rospy.loginfo(bands.file_name)
				rospy.loginfo(bands.time_string)
			print("===========================")
			period_time=((datetime.now()-lastShowTime).total_seconds())
			if ( period_time > 1) or ((platformRelease >4) and (period_time > 1)):
				process_done=True
				lastShowTime=datetime.now()
				### lowspeed loop!
				width = int(bands.raw_bands[0].width * 20 / 100)
				height = int(bands.raw_bands[0].height * 20 / 100)
				dim = (width, height)
				if band_flag==0:
					for i, axs in enumerate(fig.axes):
						imcv=bridge.imgmsg_to_cv2(bands.raw_bands[i],"16UC1")
						#im_rgb = cv2.cvtColor(imcv, cv2.COLOR_GRAY2RGB)#BGR2RGB
						new_image = cv2.resize(imcv, dim)
						axs.imshow(new_image)
				else:
					for i, axs in enumerate(fig.axes):
						imcv=bridge.imgmsg_to_cv2(bands.raw_bands[band_flag-1],"16UC1")
						axs.imshow(imcv)
				###
				#print("===========================")
				#plt.suptitle('Bands sequence: %d' %(bands.raw_bands[0].header.seq),va="top", ha="right")
				#if bands.GPS_source == "Unknown":
				#	plt.suptitle("Seq.: %d" %(bands.raw_bands[0].header.seq) + "     File Name: "+bands.file_name+"     ID: "+bands.capture_id+"     Time: " + bands.time_string + "     GPS source: %s" %(bands.GPS_source),backgroundcolor='0.75',ha="center", va="top")
				#else:
				plt.suptitle("Seq.: %d" %(bands.raw_bands[0].header.seq) + "     File Name: "+bands.file_name+"     ID: "+bands.capture_id+"     Time: " + bands.time_string + "     GPS source: %s" %(bands.GPS_source) + "     Altitude: %.2f" % (bands.altitude) ,backgroundcolor='0.75',ha="center", va="top")
				plt.draw()
				process_done=False

	except:
		rospy.signal_shutdown("You pressed Ctrl + C or closed the figure window.")

if __name__ == '__main__':
	platformRelease=int(platform.release()[0])
	print("Platform release first number: %d" % platformRelease)
	print("Close figure window to exit.")
	print('ROS node "mviewer" subscribes to "capture_data" topic to receive "micasense_ros/Multispectral" messages and show multispectral bands.')
#	try:
	lastShowTime=datetime.now()
	rospy.init_node('mviewer',anonymous=False,disable_signals=False)
	rospy.Subscriber("capture_data",Multispectral,receivecallback)
	#rospy.Subscriber("panel_data",Multispectral,callback)
	init_flag=False
	band_flag=args.band_flag
	while True:
		if rospy.is_shutdown():
			print("\nYou pressed Ctrl + C (ROS node stopped), close the figure window to exit.")
			break
		if init_flag==False:
			figure_init(band_flag)
			init_flag=True
			plt.show()
		elif not plt.fignum_exists(fig.number):
			rospy.signal_shutdown("You pressed Ctrl + C or closed the figure window.")
			break
	rospy.signal_shutdown("You pressed Ctrl + C or closed the figure window.")
	print("Shutting down")
#	except:
#		raise Exception("Error! Shutting down")
