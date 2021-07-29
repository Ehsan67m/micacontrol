#!/usr/bin/env python
from __future__ import print_function
import platform
import rospy
from sensor_msgs.msg import Image
from micasense_ros.msg import Multispectral
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import imutils


import sys, select, os
import cv2
import numpy as np
import rasterio
import rasterio.features
import rasterio.warp
import argparse
import requests
import time
from copy import copy, deepcopy
from datetime import datetime
from natsort import natsort
import json


# TODO: panel calibration, alignment quality auto checking

host = "http://192.168.1.83"
path = "/home/dji/SAR/data/"

impub=rospy.Publisher('detect_viewer', Multispectral, queue_size=10)
popub=rospy.Publisher('detect_point', Point, queue_size=10)
rospy.init_node('miprocessor', anonymous=False, disable_signals=False)


## Reference band index for alignment
REF_BAND = "RedEdge"

## To do the alignment, can be changed for images from far distance with fewer features (faster)
ITERATIONS = 100
TERMINATION = 1e-10

## To crop image after alignment
MAX_CROP_PERCENT=0.15
MAX_UNCROPED=300

## To detect changes in bands, TODO: need to find it by more data or use calibration panel data
#MIN_CHANGE=(80, 80, 50, 50)
#MAX_CHANGE=(90, 90, 110, 110)
#MIN_CHANGE=(70, 70, 50, 50)
#MAX_CHANGE=(150, 150, 150, 150)
MIN_CHANGE=(150, 110, 10, 10)
MAX_CHANGE=(180, 150, 150, 150)

## Default Realignment altitude (meters) = do alignment and generate new homographies if alignment mod is not "NO"
DEFAULT_REALI = 5

BAND_NAME = ('Blue','Green','Red','NIR','RedEdge')

parser = argparse.ArgumentParser()

#-reali REALIGNMENT -st STORAGE
parser.add_argument("-reali", "--realignment",dest = "realignment_alt", help="Redo the alignment altitude (meters). (default: %d)" % (DEFAULT_REALI), type=int)
parser.add_argument("-st", "--storage",dest = "storage_mod", help="Storage mode on Manifold memory: NO, MIN, ALL (default: MIN)", choices=['NO', 'MIN', 'ALL'])

args = parser.parse_args()
process_done=False
total_detection=0
total_image_sets=0
altitude=0.0
lastAltitude=0.0
homographie_flag=False

criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, ITERATIONS,  TERMINATION)

def get_gradient(im) :
	grad_x = cv2.Sobel(im,cv2.CV_32F,1,0,ksize=5)
	grad_y = cv2.Sobel(im,cv2.CV_32F,0,1,ksize=5)
	grad = cv2.addWeighted(np.absolute(grad_x), 0.5, np.absolute(grad_y), 0.5, 0)
	return grad

def alignImages(im2Gray, im1Gray):
	global criteria
	## Convert images to grayscale although they are from one band
	#im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
	#im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
	sz = im1Gray.shape
	height = sz[0]
	width = sz[1]
	im1Gray[im1Gray==0]=1
	im1Gray[im1Gray==255]=254
	im2Gray[im2Gray==0]=1
	im2Gray[im2Gray==255]=254
	warp_matrix = np.eye(2, 3, dtype=np.float32)
	try:
		(cc, warp_matrix) = cv2.findTransformECC (get_gradient(im1Gray),get_gradient(im2Gray),warp_matrix, cv2.MOTION_TRANSLATION, criteria)
		im2_aligned = cv2.warpAffine(im2Gray, warp_matrix, (sz[1],sz[0]), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP);
		return im2_aligned, warp_matrix
	except:
		return im2Gray, warp_matrix
		#raise #Exception ("Can not align one image")

def alignFiveBands(images, refIndex, cap_name):
	## Read reference image an band
	try:
		bands = []
		homographies = []
		mainHeight=images[refIndex].shape[0]
		mainWidth=images[refIndex].shape[1]
		print("Reference image: %s" % (BAND_NAME[refIndex]))
		print("Image shape: (",mainHeight, ",",mainWidth,")")
	
		## Align bands
		for imageIndex in range(5):
			if imageIndex != refIndex:
				print("Aligning %s band..." % (BAND_NAME[imageIndex]))
				bandImageA, h = alignImages(images[imageIndex], images[refIndex])
				#band=bandImageA[:,:,0]
				if args.storage_mod == "ALL":
					cv2.imwrite(path+ cap_name + "A_{}.tif".format(imageIndex+1),bandImageA)
				homographies.append(h)
				bands.append(bandImageA)
			else:
				#band=images[refIndex][:,:,0]
				h = None
				if args.storage_mod == "ALL":
					cv2.imwrite(path+ cap_name + "A_{}.tif".format(imageIndex+1),images[refIndex])
				homographies.append(h)
				bands.append(images[refIndex])
		if args.storage_mod !="NO":
			print("All images aligned and saved.")
		else:
			print("All images aligned.")
		return bands, homographies
	except:
		raise


def alignByHomography (im2Gray, bandHomography):
	sz = im2Gray.shape
	height = sz[0]
	width = sz[1]
	im2Gray[im2Gray==0]=1
	im2Gray[im2Gray==255]=254
	im2Gray[im2Gray==0]=1
	im2Gray[im2Gray==255]=254
	im2_aligned = cv2.warpAffine(im2Gray, bandHomography, (sz[1],sz[0]), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP);
	return im2_aligned

def alignAllByHomography (images, homographies, refIndex, cap_name):
	## Read reference image an band
	bands = []
	mainHeight=images[refIndex].shape[0]
	mainWidth=images[refIndex].shape[1]
	print("Reference image: %s" % (BAND_NAME[refIndex]))
	print("Image shape: (",mainHeight, ",",mainWidth,")")
	
	## Align bands
	for imageIndex in range(5):
		if imageIndex != refIndex:
			print("Aligning %s Band..." % (BAND_NAME[imageIndex]))
			bandA= alignByHomography (images[imageIndex], homographies[imageIndex])
			if args.storage_mod == "ALL":
				cv2.imwrite(path+cap_name+"AH_{}.tif".format(imageIndex+1),bandA)
			bands.append(bandA)
		else:
			bands.append(images[refIndex])
			if args.storage_mod == "ALL":
				cv2.imwrite(path+cap_name+"AH_{}.tif".format(imageIndex+1),images[refIndex])
	if args.storage_mod !="NO":
		print("All images aligned by using homographies and saved.")
	else:
		print("All images aligned by using homographies.")
	return bands

def findBandCropCorners(bandA):
	bandA[bandA==0]=255
	empt=bandA.min(axis=1)
	result = np. where(empt == 255)
	if len(result) > 0 and len(result[0]) > 0:
		top=result[0][0]
		height=result[0][len(result[0])-1]
		if top == 0 and height != bandA.shape[0]-1:
			top= height+1
			height= bandA.shape[0]-1
		elif top !=0 and height ==bandA.shape[0]-1:
				height=top-1
				top=0
		croppoints_V= np.array([[top],[height]])
	else:
		croppoints_V = None
	empt=bandA.min(axis=0)
	result = np. where(empt == 255)
	if len(result) > 0 and len(result[0]) > 0:
		left=result[0][0]
		width=result[0][len(result[0])-1]
		if left == 0 and width != bandA.shape[1]-1:
			left= width+1
			width= bandA.shape[1]-1
		elif left !=0 and width ==bandA.shape[1]-1:
			width=left-1
			left=0
		croppoints_H= np.array([[left],[width]])
	else:
		croppoints_H = None
	return croppoints_V, croppoints_H

def findAllCropCorners(bands, refIndex):
	croppoints_V = None
	croppoints_H = None
	for imageIndex in range(5):
		if imageIndex != refIndex:
			bandA=bands[imageIndex]
			Vpoints, Hpoints= findBandCropCorners(bandA)
			if Vpoints is not None:
				if croppoints_V is None:
					croppoints_V= np.array([Vpoints[0],Vpoints[1]])
				else:
					croppoints_V= np.concatenate((croppoints_V, np.array([Vpoints[0],Vpoints[1]])), axis=1)
			if Hpoints is not None:
				if croppoints_H is None:
					croppoints_H= np.array([Hpoints[0],Hpoints[1]])
				else:
					croppoints_H= np.concatenate((croppoints_H, np.array([Hpoints[0],Hpoints[1]])), axis=1)
	return croppoints_V, croppoints_H

def cropAlignedFiveBands(bands, refIndex, maxCrop, cap_name):
	croppoints_V = None
	croppoints_H = None
	croppoints_V, croppoints_H = findAllCropCorners(bands, refIndex)
	#print(croppoints_V)
	#print(croppoints_V.max(axis=1)[0])
	#print(croppoints_V.min(axis=1)[1])
	#print(croppoints_H)
	#print(croppoints_H.max(axis=1)[0])
	#print(croppoints_H.min(axis=1)[1])
	total_uncroped=10000;
	croping=0
	bandsCropped=deepcopy(bands)
	print("Cropping all images...")
	while (total_uncroped>MAX_UNCROPED) and (croping<maxCrop):
		total_uncroped=0
		croping=croping+1
		for imageIndex in range(5):
			if imageIndex != refIndex:
				if (croppoints_H is not None) and (croppoints_V is not None):
					bandsCropped[imageIndex]=bands[imageIndex][croppoints_V.max(axis=1)[0]+croping:croppoints_V.min(axis=1)[1]-croping, croppoints_H.max(axis=1)[0]+croping:croppoints_H.min(axis=1)[1]-croping]
					bandsCropped[imageIndex][bandsCropped[imageIndex]==0]=255
				result = np. where(bandsCropped[imageIndex] == 255)
				if len(result) > 0 and len(result[0]) > 0:
					#print(len(result[0]))
					total_uncroped=total_uncroped+len(result[0])
				#else:
					#print("nothing")
				bandsCropped[imageIndex][bandsCropped[imageIndex]==255]=0
			elif (croppoints_H is not None) and (croppoints_V is not None):
				bandsCropped[imageIndex]=bands[refIndex][croppoints_V.max(axis=1)[0]+croping:croppoints_V.min(axis=1)[1]-croping, croppoints_H.max(axis=1)[0]+croping:croppoints_H.min(axis=1)[1]-croping]
			if args.storage_mod == "ALL":
				cv2.imwrite(path + cap_name + "C_A_{}.tif".format(imageIndex+1), bandsCropped[imageIndex])
	if args.storage_mod !="NO":
		print("All images cropped and saved.")
	else:
		print("All images cropped.")
	return bandsCropped

def detectChanges(bandsCropped, minChange, maxChange):
	#TODO: Need to be changed on the manifold (it works fine on ubuntu 18 not ubuntu 16)
	global altitude
	detected_point=Point()
	kernel1 = np.ones((3,3), np.uint8)
	kernel2 = np.ones((15,15), np.uint8)

	blueNIR=(bandsCropped[3]-bandsCropped[0])
	blueNIR[blueNIR <minChange[0]]=0
	blueNIR[blueNIR >maxChange[0]]=0
	blueNIR[blueNIR >0]=60
	blueNIR = cv2.erode( blueNIR , kernel1, iterations=1)
	blueNIR = cv2.erode( blueNIR , kernel1, iterations=1)
	blueNIR = cv2.erode( blueNIR , kernel1, iterations=1)
	blueNIR = cv2.dilate( blueNIR , kernel2, iterations=1)

	greenNIR=(bandsCropped[3]-bandsCropped[1])
	greenNIR[greenNIR<minChange[1]]=0
	greenNIR[greenNIR>maxChange[1]]=0
	greenNIR[greenNIR>0]=60
	greenNIR = cv2.erode(greenNIR, kernel1, iterations=1)
	greenNIR = cv2.erode(greenNIR, kernel1, iterations=1)
	greenNIR = cv2.erode(greenNIR, kernel1, iterations=1)
	greenNIR = cv2.dilate(greenNIR, kernel2, iterations=1)

	redNIR=(bandsCropped[3]-bandsCropped[2])
	redNIR[redNIR<minChange[2]]=0
	redNIR[redNIR>maxChange[2]]=0
	redNIR[redNIR>0]=60
	redNIR = cv2.erode(redNIR, kernel1, iterations=1)
	redNIR = cv2.erode(redNIR, kernel1, iterations=1)
	redNIR = cv2.dilate(redNIR, kernel2, iterations=1)

	edgeNIR=(bandsCropped[3]-bandsCropped[4])
	edgeNIR[edgeNIR<minChange[3]]=0
	edgeNIR[edgeNIR>maxChange[3]]=0
	edgeNIR[edgeNIR>0]=60
	edgeNIR = cv2.erode(edgeNIR, kernel1, iterations=1)
	edgeNIR = cv2.erode(edgeNIR, kernel1, iterations=1)
	edgeNIR = cv2.erode(edgeNIR, kernel1, iterations=1)
	edgeNIR = cv2.dilate(edgeNIR, kernel2, iterations=1)

	AND1=redNIR+edgeNIR
	AND1[AND1<100]=0
	AND1[AND1>0]=60
	AND1 = cv2.erode(AND1, kernel1, iterations=1)
	AND1 = cv2.dilate(AND1, kernel2, iterations=1)

	AND2=redNIR+blueNIR
	AND2[AND2<100]=0
	AND2[AND2>0]=60
	AND2 = cv2.erode(AND2, kernel1, iterations=1)
	AND2 = cv2.dilate(AND2, kernel2, iterations=1)

	AND3=redNIR+greenNIR
	AND3[AND3<100]=0
	AND3[AND3>0]=60
	AND3 = cv2.erode(AND3, kernel1, iterations=1)
	AND3 = cv2.dilate(AND3, kernel2, iterations=1)

	detected=AND1+AND2+AND3
	detected[detected>130]=60
	detected = cv2.erode(detected, kernel1, iterations=1)
	detected = cv2.dilate(detected, kernel2, iterations=1)


	blueNIR=(bandsCropped[3]-bandsCropped[0])
	blueNIR[blueNIR <minChange[0]]=0
	blueNIR[blueNIR >maxChange[0]]=0
	blueNIR[blueNIR >0]=60
	blueNIR = cv2.erode( blueNIR , kernel1, iterations=1)
	blueNIR = cv2.erode( blueNIR , kernel1, iterations=1)
	blueNIR = cv2.erode( blueNIR , kernel1, iterations=1)
	blueNIR = cv2.dilate( blueNIR , kernel2, iterations=1)

	greenNIR=(bandsCropped[3]-bandsCropped[1])
	greenNIR[greenNIR<minChange[1]]=0
	greenNIR[greenNIR>(maxChange[1]-30)]=0
	greenNIR[greenNIR>0]=60
	greenNIR = cv2.erode(greenNIR, kernel1, iterations=1)
	greenNIR = cv2.erode(greenNIR, kernel1, iterations=1)
	greenNIR = cv2.erode(greenNIR, kernel1, iterations=1)
	greenNIR = cv2.dilate(greenNIR, kernel2, iterations=1)

	redNIR=(bandsCropped[3]-bandsCropped[2])
	redNIR[redNIR<minChange[2]]=0
	redNIR[redNIR>maxChange[2]]=0
	redNIR[redNIR>0]=60
	redNIR = cv2.erode(redNIR, kernel1, iterations=1)
	redNIR = cv2.erode(redNIR, kernel1, iterations=1)
	redNIR = cv2.dilate(redNIR, kernel2, iterations=1)

	edgeNIR=(bandsCropped[3]-bandsCropped[4])
	edgeNIR[edgeNIR<minChange[3]]=0
	edgeNIR[edgeNIR>maxChange[3]]=0
	edgeNIR[edgeNIR>0]=60
	edgeNIR = cv2.erode(edgeNIR, kernel1, iterations=1)
	edgeNIR = cv2.erode(edgeNIR, kernel1, iterations=1)
	edgeNIR = cv2.erode(edgeNIR, kernel1, iterations=1)
	edgeNIR = cv2.dilate(edgeNIR, kernel2, iterations=1)

	AND1=redNIR+edgeNIR
	AND1[AND1<100]=0
	AND1[AND1>0]=60
	AND1 = cv2.erode(AND1, kernel1, iterations=1)
	AND1 = cv2.dilate(AND1, kernel2, iterations=1)

	AND2=redNIR+blueNIR
	AND2[AND2<100]=0
	AND2[AND2>0]=60
	AND2 = cv2.erode(AND2, kernel1, iterations=1)
	AND2 = cv2.dilate(AND2, kernel2, iterations=1)

	AND3=redNIR+greenNIR
	AND3[AND3<100]=0
	AND3[AND3>0]=60
	AND3 = cv2.erode(AND3, kernel1, iterations=1)
	AND3 = cv2.dilate(AND3, kernel2, iterations=1)

	detected2=AND1+AND2+AND3
	detected2[detected2>130]=60
	#detected2 = cv2.erode(detected, kernel1, iterations=1)
	#detected2 = cv2.dilate(detected, kernel2, iterations=1)

	detected3= abs(detected - detected2)
	detected3 = cv2.erode(detected3, kernel2, iterations=1)
	detected3 = cv2.dilate(detected3, kernel2, iterations=1)
	detected3[detected3<60]=0
	detected3[detected3>50]=60
	detected3 = cv2.convertScaleAbs(detected3, alpha=0.03)
	contours = cv2.findContours(detected3,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours=imutils.grab_contours(contours)
	for c in contours:
		# calculate moments for each contour
		M = cv2.moments(c)

		# calculate x,y coordinate of center
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
		else:
			cX, cY = 0, 0
		print(cX,cY)
		
		cv2.circle(detected3, (cX, cY), 25, (255, 255, 255), -1)
		cv2.circle(detected3, (cX, cY), 20, (0, 0, 0), -1)
		cv2.circle(detected3, (cX, cY), 5, (255, 255, 255), -1)
		detected3[detected3<70]=0
		detected3[detected3>50]=60
		detected_point.x=cX
		detected_point.y=cY
		detected_point.z=altitude
		popub.publish(detected_point)

	return blueNIR,greenNIR,redNIR,edgeNIR, detected3

def check_inputs():
	try:
		if (args.realignment_alt is not None) and (args.realignment_alt<3):
			raise Exception ("The realignment altitude should be a positive value and (>= 3), %d is not acceptable." % (args.realignment_alt))
		elif (args.realignment_alt is None):
			args.realignment_alt=DEFAULT_REALI

		if (args.storage_mod is None):
			#args.storage_mod = "ALL"
			args.storage_mod = "NO"
			#args.storage_mod = "MIN"
	except:
		raise

def listener_new():
	global process_done
	msg=rospy.wait_for_message('capture_data', Multispectral)
	if process_done==False:
		callback(msg)
	else:
		return

def callback(captured_bands):
	global lastAltitude
	global altitude
	global total_detection
	global total_image_sets
	global previous_homographie
	global homographie_flag
	global process_done
	result_images=Multispectral()
	bridge=CvBridge()
	bands =None
	GPS_source="Unknown"
	process_done=True
	if not rospy.is_shutdown():
		os.system('cls' if os.name== 'nt' else 'clear')
		GPS_source=captured_bands.GPS_source
		altitude=captured_bands.altitude
		result_images.capture_id=captured_bands.capture_id
		result_images.time_string=captured_bands.time_string
		result_images.file_name=captured_bands.file_name
		result_images.altitude=captured_bands.altitude
		result_images.latitude=captured_bands.latitude
		result_images.longitude=captured_bands.longitude
		imgs=[]
		for imageIndex in range(5):
			result_images.raw_bands.append(captured_bands.raw_bands[imageIndex])
			img=bridge.imgmsg_to_cv2(captured_bands.raw_bands[imageIndex],"16UC1")
			imgs.append(img)
		if captured_bands.raw_bands[0].header.seq is not None:
			print('Saving the results at path "'+path+'" : '+ args.storage_mod)
			print("Processing sets number: %d" % captured_bands.raw_bands[0].header.seq)
		if (GPS_source=="Unknown") and (homographie_flag==False):
			print("The altitude is Unknown, Do new alignment:") 
			bands, previous_homographie = alignFiveBands(imgs, BAND_NAME.index(REF_BAND),captured_bands.capture_id)
			homographie_flag=True
		elif (GPS_source != "Unknown") and (abs(lastAltitude-altitude) > args.realignment_alt):
			## read GPS data (altitude) to align again and create Homography
			print("The altitude changed more than %dm, Do new alignment:" % args.realignment_alt) 
			bands, previous_homographie = alignFiveBands(imgs, BAND_NAME.index(REF_BAND),captured_bands.capture_id)
			homographie_flag=True
		else:
			## using the homography
			print("The altitude changes was bellow %dm, Use previous alignment data:" % args.realignment_alt)
			bands= alignAllByHomography(imgs, previous_homographie, BAND_NAME.index(REF_BAND),captured_bands.capture_id)

  	
		if bands is not None:
			## Crop aligned images
			print("Cropping images...")
			maxCrop= captured_bands.raw_bands[BAND_NAME.index(REF_BAND)].height*MAX_CROP_PERCENT
			bandsCropped=None
			bandsCropped =cropAlignedFiveBands(bands, BAND_NAME.index(REF_BAND), maxCrop,captured_bands.capture_id)
			if any(x is not None for x in bandsCropped):
			#if bandsCropped is not None:
				if ((args.storage_mod == "ALL") or (args.storage_mod == "MIN")):
					cv2.imwrite(path+ captured_bands.capture_id +"_BGR.tif", cv2.merge([bandsCropped[0],bandsCropped[1],bandsCropped[2]]))
				bgr_image=cv2.merge([bandsCropped[0],bandsCropped[1],bandsCropped[2]])
				try:
					msg=bridge.cv2_to_imgmsg(bgr_image,"16UC3")
				except:
					msg=bridge.cv2_to_imgmsg(bands[0],"16UC1")
				result_images.raw_bands.append(msg)
			
				print("Detecting in images...")
				blueNIR , greenNIR , redNIR , edgeNIR, detected = detectChanges(bandsCropped, MIN_CHANGE, MAX_CHANGE)
				if args.storage_mod == "ALL":
					cv2.imwrite(path+ captured_bands.capture_id +"_blueNIR.tif", blueNIR)
					cv2.imwrite(path+ captured_bands.capture_id +"_greenNIR.tif", greenNIR)
					cv2.imwrite(path+ captured_bands.capture_id +"_redNIR.tif", redNIR)
					cv2.imwrite(path+ captured_bands.capture_id +"_edgeNIR.tif", edgeNIR)
					cv2.imwrite(path+ captured_bands.capture_id +"_detected.tif", detected)
					cv2.imwrite(path+ captured_bands.capture_id +"_detectedBGR.tif", cv2.merge([(bandsCropped[0]-detected), (bandsCropped[1]-detected), (bandsCropped[2]-detected)]))
				elif args.storage_mod == "MIN":
					cv2.imwrite(path+ captured_bands.capture_id +"_detectedBGR.tif", cv2.merge([(bandsCropped[0]-detected), (bandsCropped[1]-detected), (bandsCropped[2]-detected)]))

				detected_image=cv2.merge([(bandsCropped[0]-detected), (bandsCropped[1]-detected), (bandsCropped[2]-detected)])
				msg=bridge.cv2_to_imgmsg(detected_image,"16UC3")
				result_images.raw_bands.append(msg)
				print("Result published!")
				impub.publish(result_images)
			else:
				print("Error during cropping")
		lastAltitude=altitude
	process_done=False


if __name__ == '__main__':
	
	platformRelease=int(platform.release()[0])
	print("Platform release first number: %d" % platformRelease)
	print('ROS node "miprocessor" subscribes to the "capture_data" topic to receive the "micasense_ros/Multispectral" messages. Then publish the image processing results (images as "micasense_ros/Multispectral") on the "detect_viewer" topic and detected point on the "detect_point" topic (point as "geometry_msgs/point" with x,y as image pixels and z as altitude).')
	## Check validation of inputs
	check_inputs()
	while True:
		listener_new()
		if rospy.is_shutdown():
			print("\nYou pressed Ctrl + C (ROS node stopped).")
			break
		#os.system('cls' if os.name== 'nt' else 'clear')	

	rospy.signal_shutdown("You pressed Ctrl + C.")
	print("Shutting down")
