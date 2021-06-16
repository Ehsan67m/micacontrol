host = "http://192.168.1.83"
path = "/home/dji/SAR/data/"

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

# TODO: panel calibration, alignment quality auto checking, reporting detected location on ros topic, switch to manifold memory in case of full SD card or no SD card


## Reference band index for alignment
REF_BAND = "RedEdge"

## To do the alignment, can be changed for images from far distance with fewer features (faster)
MAX_FEATURES = 10000
GOOD_MATCH_PERCENT = 0.1

## To crop image after alignment
MAX_CROP_PERCENT=0.15
MAX_UNCROPED=300

## To detect changes in bands, TODO: need to find it by more data or use calibration panel data
MIN_CHANGE=(70, 70, 50, 50)
MAX_CHANGE=(150, 150, 110, 110)

## Default timer period (second) = period between each capture (in addition to the image processing required time if it is active)
DEFAULT_TIMER = 0
## Default operating altitude (meters) = start capturing only if it is above the operating altitude
DEFAULT_OA = 2
## Default operating altitude tolerance (meters)
DEFAULT_OATOLE = 1
## Default Realignment altitude (meters) = do alignment and generate new homographies if alignment mod is not "NO"
DEFAULT_REALI = 5
## Default crroping flag (0 or 1) = if 1, do cropping after the alignment if alignment mod is not "NO"
DEFAULT_CROPPING= 1
## Default panel detection flag (0 or 1) = if 1, no capture will repurt until detecting the panel (first capture would be the panel), or until the panel_timer (second) is passed
DEFAULT_PANEL = 0
DEFAULT_PANEL_TIMER = 10

BAND_NAME = ('Blue','Green','Red','NIR','RedEdge')

parser = argparse.ArgumentParser()

#-adr ADDRESS -t TIMER -oa OPERATING_ALTITUDE -oatole OA_TOLERANCE -d DETECTION -ali ALIGNMENT -reali REALIGNMENT -cr CROPPING -p PRINT -cpl CAPTURE_PANEL -f FORMAT -s STORAGE
parser.add_argument("-adr", "--address", dest = "image_path", help="If provided, alignment, cropping, and detection will be used for images in address (instead of the camera)")
parser.add_argument("-ti", "--timer", dest = "timer_period", help="Period between each captures (seconds). (default: %d)" % (DEFAULT_TIMER),type=int)
parser.add_argument("-oa", "--oalt", dest = "operating_alt", help="Aircraft operating altitude above ground (meters). (default: %d)" % (DEFAULT_OA), type=int)
parser.add_argument("-oatole", "--oatole",dest ="oalt_tolerance", help="Tolerance altitude (meters). (default: %d)" % (DEFAULT_OATOLE), type=int)
parser.add_argument("-de", "--detection",dest = "detection_mod", default = 0, help="Detection modes: 1(=True), 0(=False) (default: 0)",type=int, choices=range(2))
parser.add_argument("-ali", "--alignment",dest = "alignment_mod", default = "ORB", help="Alignment modes: ORB, NO (default: ORB)", choices=['ORB', 'NO'])
parser.add_argument("-reali", "--realignment",dest = "realignment_alt", help="Redo the alignment altitude (meters). (default: %d)" % (DEFAULT_REALI), type=int)
parser.add_argument("-cr", "--crop",dest = "cropping_mod", help="Cropping: 1 (=True), 0(=False). (default: %d)" % (DEFAULT_CROPPING),type=int, choices=range(2))
parser.add_argument("-pr", "--print",dest = "print_mod", default = 0, help="Print information: 1 (=True), 0(=False) (default: 0)",type=int, choices=range(2))
parser.add_argument("-cpl", "--capturepanel",dest = "detect_panel", help="Cpture calibration panel first: 1(=True), 0(=False). (default: %d)" % (DEFAULT_PANEL),type=int, choices=range(2))
parser.add_argument("-fo", "--format",dest = "format_mod", help="Format camera memory before start: 1 (=True), 0(=False)",type=int, choices=range(2))
parser.add_argument("-st", "--storage",dest = "storage_mod", default = "NO", help="Storage mode on Manifold memory: NO, MIN, ALL (default: NO)", choices=['NO', 'MIN', 'ALL'])


args = parser.parse_args()

def check_connection():
	attempt=3
	while (attempt>0):
		try:
			resp = requests.get(host + "/status", timeout=3)
			if args.print_mod:
				print("SD card status: " + resp.json()["sd_status"])
				if resp.json()["dls_status"] != "Ok":
					print("DLS status: " + resp.json()["dls_status"])
				else:
					print("GPS warn: " + str(resp.json()["gps_warn"]))
					if resp.json()["utc_time_valid"] != True:
						print("UTC time is not valid.")
					else:
						print("UTC time: " + str(resp.json()["utc_time"]))
			return resp.json()
		except requests.exceptions.RequestException:
			attempt-=1
			print ("Camera is not connected...")

def capture_bands(resp):
  try:
	## Sending the capture command, getting the capture id
	capture_id = requests.get(host + "/capture", timeout=3).json()["id"]
	#print(capture_id)
	startTime = datetime.now()
	## Waiting for capture complete status
	while str(requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()["status"]) != "complete" :
		tdelta = datetime.now() - startTime
		if(tdelta.total_seconds() >= 5):
			## Check if the problem is becouse of the panel_detection
			detect_panel_status = requests.get(host + "/detect_panel", timeout=3).json()["detect_panel"]
			if detect_panel_status== True :
				detect_panel_status = requests.post(host + "/detect_panel", json={"abort_detect_panel":True}, timeout=3).json()["detect_panel"]
				print("Panel detection was True, now it is %s. Capturing new images..." % str(detect_panel_status) )
				capture_id = requests.get(host + "/capture", timeout=3).json()["id"]
				startTime = datetime.now()
			else:
				#camera_powerdown_ready()
				raise Exception ("Capturing is not possible.")
	if (args.alignment_mod != "NO"):
		capture_data = requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()
		if resp["sd_status"]=="Ok":
			cap_name = str(capture_data["raw_storage_path"]["1"])
			a=0
			b=0
		else:
			cap_name = str(capture_data["raw_cache_path"]["1"])
			a=6
			b=2
		if cap_name != "None":
			cap_name=cap_name[7-a:len(cap_name)-(6-b)]
			cap_name=cap_name.replace("/","-")
			## Save captured images (in manifold)
			for n in range(1,6):
				im = requests.get(host + str(capture_data["raw_cache_path"][str(n)]), stream=True)
				with open(path + "band_%d.tif" % (n), 'wb') as f:
			        	for chunk in im.iter_content(10240):
		               			f.write(chunk)
		else:
			cap_name = None
	else:
		cap_name = None
	return cap_name
  except requests.exceptions.RequestException:
	raise Exception ("Camera did not respond to the capture command!")
  except:
	raise
	
def capture_panel(resp):
  try:
  	## Sending the capture panel command, getting the capture id
	capture_id = requests.post(host + "/capture/detect_panel", json={"detect_panel":True}, timeout=3).json()["id"]
	detect_panel_status = requests.get(host + "/detect_panel", timeout=3).json()["detect_panel"]
	#print(capture_id)
	startTime = datetime.now()
	panel_captured = False
	print("Searching for the panel in next {} seconds...".format(DEFAULT_PANEL_TIMER))
	## Waiting for capture complete status
	while (str(requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()["status"]) != "complete") and (detect_panel_status==True) :
		detect_panel_status = requests.get(host + "/detect_panel", timeout=3).json()["detect_panel"]
		tdelta = datetime.now() - startTime
		capture_id = requests.get(host + "/capture", timeout=3).json()["id"]
		if(tdelta.total_seconds() >= DEFAULT_PANEL_TIMER):
			confirmation = str(raw_input("Panel didn't detected after %ds, do you want to searching for another cyle? (y/n) " % DEFAULT_PANEL_TIMER)).lower().strip()
			while (confirmation[:1] != 'y') and (confirmation[:1] != 'n'):
				confirmation = str(raw_input("Invalid input! do you want to searching for another cyle? (y/n) ")).lower().strip()
			if (confirmation[:1] == 'y'):
				print("Continue searching for the panel...")
				startTime = datetime.now()
			else:
				## Turn off the panel detection
				detect_panel_status = requests.post(host + "/detect_panel", json={"abort_detect_panel":True}, timeout=3).json()["detect_panel"]
				print("Panel detection was True, but the panel is not detected after %d seconds. Now it is %s." % (DEFAULT_PANEL_TIMER , str(detect_panel_status)) )
				panel_captured = False
	if (str(requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()["status"]) == "complete"):
		panel_captured = True
		print("Panel detected.")
		capture_data = requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()
		if resp["sd_status"]=="Ok":
			cap_name = str(capture_data["raw_storage_path"]["1"])
			a=0
			b=0
		else:
			cap_name = str(capture_data["raw_cache_path"]["1"])
			a=6
			b=2
		if cap_name != "None":
			cap_name=cap_name[7-a:len(cap_name)-(6-b)]
			cap_name=cap_name.replace("/","-")
			## Save captured images (in manifold)
			for n in range(1,6):
				im = requests.get(host + str(capture_data["raw_cache_path"][str(n)]), stream=True)
				with open(path + "panel_%d.tif" % (n), 'wb') as f:
	        			for chunk in im.iter_content(10240):
               					f.write(chunk)
		else:
			cap_name= None
		return cap_name , panel_captured
	else:
		cap_name= None
		return cap_name , panel_captured
  except requests.exceptions.RequestException:
	raise Exception ("Camera did not respond to the capture panel command!")
  except:
	#camera_powerdown_ready()
	raise



def alignImages(im1, im2):
   try:
	## Convert images to grayscale although they are from one band
	im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
	im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)

	## Detect ORB features (license free detector) and compute descriptors
	orb = cv2.ORB_create(MAX_FEATURES)
	keypoints1, descriptors1 = orb.detectAndCompute(im1Gray, None)
	keypoints2, descriptors2 = orb.detectAndCompute(im2Gray, None)

	## Match features
	matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
	matches = matcher.match(descriptors1, descriptors2, None)

	## Sort matches by score
	matches.sort(key=lambda x: x.distance, reverse=False)

	## Remove not so good matches
	numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
	matches = matches[:numGoodMatches]
  
	## Draw top matches
	#imMatches = cv2.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
	#cv2.imwrite(path+"matches.jpg", imMatches)

	## Extract location of good matches
	points1 = np.zeros((len(matches), 2), dtype=np.float32)
	points2 = np.zeros((len(matches), 2), dtype=np.float32)

	for i, match in enumerate(matches):
		points1[i, :] = keypoints1[match.queryIdx].pt
		points2[i, :] = keypoints2[match.trainIdx].pt

	## Find homography
	h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)

	## Use homography
	height, width, channels = im2.shape
	im1Reg = cv2.warpPerspective(im1, h, (width, height))

	return im1Reg, h
   except:
	raise Exception ("Can not align one image")

def loadFiveBand(imgPath):
	## imgPath = image address in memory contating its name without .tif
	## Read all bands
   try:
	images = []
	for imageIndex in range(5):
		imFilename = imgPath+"_{}.tif".format(imageIndex+1)
		im = cv2.imread(imFilename)
		if args.print_mod:
			print("Reading %s band: " % (BAND_NAME[imageIndex]), imFilename)
		im[im==0]=1
		im[im==255]=254
		images.append(im)
	return images
   except:
	raise Exception ("Can not load images")


def alignFiveBands(images, refIndex, cap_name):
	## Read reference image an band
   try:
	bands = []
	homographies = []
	mainHeight=images[refIndex].shape[0]
  	mainWidth=images[refIndex].shape[1]
	if args.print_mod:
		print("Reference image: %s" % (BAND_NAME[refIndex]))
		print("Image shape: (",mainHeight, ",",mainWidth,")")
	
	## Align bands
	for imageIndex in range(5):
		if imageIndex != refIndex:
			if args.print_mod:
				print("Aligning %s band..." % (BAND_NAME[imageIndex]))
			bandImageA, h = alignImages(images[imageIndex], images[refIndex])
			band=bandImageA[:,:,0]
			if args.storage_mod == "ALL":
				cv2.imwrite(path+ cap_name + "A_{}.tif".format(imageIndex+1),bandImageA)
			homographies.append(h)
			bands.append(band)
		else:
			band=images[refIndex][:,:,0]
			h = None
			if args.storage_mod == "ALL":
				cv2.imwrite(path+ cap_name + "A_{}.tif".format(imageIndex+1),images[refIndex])
			homographies.append(h)
			bands.append(band)
	if args.print_mod:
		if args.storage !="NO":
			print("All images aligned and saved.")
		else:
			print("All images aligned.")
	return bands, homographies
   except:
	raise Exception ("Can not align all images")


def alignByHomography (bandImage, bandHomography, refImage):
	height, width, channels = refImage.shape
	alignedImg = cv2.warpPerspective(bandImage, bandHomography, (width, height))
	bandA=alignedImg[:,:,0]
	return bandA

def alignAllByHomography (images, homographies, refIndex, cap_name):
	## Read reference image an band
	bands = []
	mainHeight=images[refIndex].shape[0]
  	mainWidth=images[refIndex].shape[1]
	if args.print_mod:
		print("Reference image: %s" % (BAND_NAME[refIndex]))
		print("Image shape: (",mainHeight, ",",mainWidth,")")
	
	## Align bands
	for imageIndex in range(5):
		if imageIndex != refIndex:
			if args.print_mod:
				print("Aligning %s Band..." % (BAND_NAME[imageIndex]))
			bandA= alignByHomography (images[imageIndex], homographies[imageIndex], images[refIndex])
			if args.storage_mod == "ALL":
				cv2.imwrite(path+cap_name+"AH_{}.tif".format(imageIndex+1),bandA)
			bands.append(bandA)
		else:
			band=images[refIndex][:,:,0]
			bands.append(band)
			if args.storage_mod == "ALL":
				cv2.imwrite(path+cap_name+"AH_{}.tif".format(imageIndex+1),images[refIndex])
	if args.print_mod:
		if args.storage !="NO":
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
		else:
			if top !=0 and height ==bandA.shape[0]-1:
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
		else:
			if left !=0 and width ==bandA.shape[1]-1:
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
	if args.print_mod:
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
	if args.print_mod:
		if args.storage !="NO":
			print("All images cropped and saved.")
		else:
			print("All images cropped.")
	return bandsCropped

def detectChanges(bandsCropped, minChange, maxChange):
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
	detected[detected>0]=60
	detected = cv2.erode(detected, kernel1, iterations=1)
	detected = cv2.dilate(detected, kernel2, iterations=1)

	return blueNIR,greenNIR,redNIR,edgeNIR, detected

def check_inputs():
  try:
	if ((args.image_path is not None) and ((args.timer_period is not None) or (args.operating_alt is not None) or (args.oalt_tolerance is not None) or (args.detect_panel is not None) or (args.format_mod is not None) or (args.realignment_alt is not None))):
		raise Exception ("The Address option is priveded alongside other capturing options. Image processing is possible only either from the memory OR the camera, not both.")

	if (args.timer_period is not None) and (args.timer_period<0):
		raise Exception ("The timer period should be a positive value, %d is not acceptable." % args.timer_period)
	elif (args.timer_period is None):
		args.timer_period=DEFAULT_TIMER

	if (args.operating_alt is not None) and (args.operating_alt<0):
		raise Exception ("The operating altitude tolerance should be a positive value, %d is not acceptable." % (args.operating_alt))
	elif (args.operating_alt is None):
		args.operating_alt=DEFAULT_OA

	if (args.oalt_tolerance is not None):
		if (args.operating_alt is not None):
			if (args.oalt_tolerance>=args.operating_alt):
				raise Exception ("The operating altitude tolerance should be less than the operating altitude (%d), %d is not acceptable." % (args.operating_alt, args.oalt_tolerance))
		if (args.oalt_tolerance<0): 
			raise Exception ("The operating altitude tolerance should be a positive value, %d is not acceptable." % (args.oalt_tolerance))
	elif (args.oalt_tolerance is None):
		args.oalt_tolerance=DEFAULT_OATOLE

	if (args.realignment_alt is not None) and (args.realignment_alt<0):
		raise Exception ("The realignment altitude should be a positive value, %d is not acceptable." % (args.realignment_alt))
	elif (args.realignment_alt is not None) and (args.alignment_mod== "NO"):
		raise Exception ("Realignment is not possible when alignment mode is (NO)!")
	elif (args.realignment_alt is None):
		args.realignment_alt=DEFAULT_REALI
	
	if (args.detection_mod == 1):
		if (args.alignment_mod== "NO"):
			raise Exception ("Detection is not possible when alignment mode is (NO)!")
		elif (args.cropping_mod is not None):
			if (args.cropping_mod ==0):
				raise Exception ("Detection is not possible when cropping mode is 0!")
		else:
			args.cropping_mod = 1

	if (args.cropping_mod is not None) and (args.alignment_mod== "NO"):
		if args.cropping_mod == 1:
			raise Exception ("Cropping is not possible when alignment mode is (NO)!")
	elif (args.cropping_mod is None):
		if (args.alignment_mod != "NO"):
			args.cropping_mod=DEFAULT_CROPPING
		else:
			args.cropping_mod = 0

	if (args.detect_panel is None):
		args.detect_panel= DEFAULT_PANEL

	if (args.image_path is not None):
		if args.storage_mod != "MIN":
			args.storage_mod="ALL"
	
	#print( "Address: {}, Timer period: {}, Operating Alt: {}, OAlt Tolerance: {}, Detection: {}, Alignment: {}, Realignment Alt: {}, Cropping: {}, Print: {}, Detect panel: {}, Format: {}, Storage: {}.".format(
	#		args.image_path,
	#	        args.timer_period,
	#	        args.operating_alt,
	#	        args.oalt_tolerance,
	#		args.detection_mod,
	#	        args.alignment_mod,
	#		args.realignment_alt,
	#		args.cropping_mod,
	#	        args.print_mod,
	#	        args.detect_panel,
	#	        args.format_mod,
	#	        args.storage_mod
	#	        ))
  except:
	raise

def camera_powerdown_ready():
  try:
	ready_for_power_down= False
	while not (ready_for_power_down):
		ready_for_power_down = requests.post(host + "/powerdownready", json={"power_down":True}, timeout=3).json()["ready_for_power_down"]
	print("Camera is ready to be powered down.")
  except requests.exceptions.RequestException:
	print ("Camera did not respond to the power down command!")
  except:
	print ("Can not make the camera ready to be powered down!")

def reformat_sd_card(force_flag):
  try:
	reformat_status = "fail"
	if force_flag == False:
		confirmation = str(raw_input("Are you sure to reformat the SD card before start? (y/n) ")).lower().strip()
		while (confirmation[:1] != 'y') and (confirmation[:1] != 'n'):
			confirmation = str(raw_input("Invalid input! Are you sure to reformat the SD card before start? (y/n) ")).lower().strip()
	if (confirmation[:1] == 'y') or (force_flag):
		print("Reformating the SD card...")
		while (reformat_status != "success"):
			reformat_status = str(requests.post(host + "/reformatsdcard", json={"erase_all_data":True}, timeout=120).json()["reformat_status"])
		print("The SD card reformated.")
  except requests.exceptions.RequestException:
	print ("Camera did not respond to the format command!")
  except:
	camera_powerdown_ready()
	print ("Can not reformat the SD card!")

if __name__ == '__main__':
   ## Check validation of inputs
   check_inputs()
   
   total_captures=0
   lastAltitude=0
   altitude=0
   lastCaptureTime=datetime.now()
   capturing = False
   format_asked= False
   panel_captured = False
   capture_panel_implimented = False
   start_time=datetime.now()

   while (True):
	os.system('cls' if os.name== 'nt' else 'clear')
	cap_name = None
	pan_name = None
	captured = False
	aligned = False
	cropped = False
	print("Press Enter to Exit (make camera ready to shotdown)")
	## check connection, capture and load from ethernet connector (and save to path)
	resp=check_connection()
	if (resp is not None) and (args.image_path is None):
		print("SD free space:{:.2f} GB, GPS warn:{}, Alignment:{}, Cropping:{}, Detection:{}, Storage mode:{}, Panel Detection:{}, Panel detected:{}".format(
			resp["sd_gb_free"],
			str(resp["gps_warn"]),
			args.alignment_mod,
			["False","True"][args.cropping_mod],
			["False","True"][args.detection_mod],
			args.storage_mod,
			["False","True"][args.detect_panel],
			panel_captured
			))
		print("Total captured sets: %d" % total_captures)
		print("Current Altitude: %.2f" % altitude)
		print("Last Capture Altitude: %.2f" % lastAltitude)
		## Reformat SD card if requested
		if (args.format_mod is not None) and (format_asked == False):
			if args.format_mod:
				if resp["sd_status"] != "NotPresent":
					reformat_sd_card(False)
					format_asked = True
				else:
					print ("Reformat was requested, but SD card is not present (or it is unmounted)!")
		## If SD card is full
		if ((resp["sd_status"]=="Full") or (resp["sd_warn"]==True)) and resp["sd_status"] != "NotPresent":
			print("Free space in the SD card is %s (GB), reformatting is in progress..." % str(resp["sd_gb_free"])) 
			## Check if the code has been started recently
			if (datetime.now()-start_time).total_seconds()<5:
				reformat_sd_card(False)
				format_asked = True
			else:
				## TODO: switch to Manifold memory?
				#reformat_sd_card(True)
				#camera_powerdown_ready()
				#raise Exception ("SD card is full!")
				print("SD card is full!")
		elif (resp["sd_status"]=="NotPresent"):
			## TODO: switch to manifold memory?
			print ("SD card is not present (or it is unmounted)!")

				
		## Capture panel if requested during the DEFAULT_PANEL_TIMER time
		if (args.detect_panel is not None) and (capture_panel_implimented== False):
			if args.detect_panel:
				pan_name , panel_captured = capture_panel(resp)
				capture_panel_implimented= True
				if pan_name is not None:
					captured = True
					total_captures+=1
		## If positioning was not accurate
		if (resp["dls_status"]!="Ok") or (resp["gps_warn"]!=True) or (resp["p_acc"]>=10):
			altitude=-20
			capturing=True
		else:
			## Checking the operating altitude for capturing
			altitude=resp["alt_agl"]
			if altitude >= (args.operating_alt + args.oalt_tolerance):
				capturing= True
			elif altitude <= (args.operating_alt - args.oalt_tolerance):
				capturing= False
	  	if capturing:
			## Capture if position is not accurate or it is above the operating range
			cap_name = capture_bands(resp)
			if cap_name is not None:
	  			images=loadFiveBand(path+"band")
			total_captures+=1
			if altitude != -20:
				lastAltitude=altitude
			lastCaptureTime=datetime.now()
			if cap_name is not None:
				captured = True
	elif (args.image_path is not None):
	   try:
		print("Load images from memory is requested")
		images=loadFiveBand(args.image_path)
		b= args.image_path.index("/", -10)+1
		cap_name=args.image_path[b:]
		#print(b)
		capturing= False
		captured = True
	   except ValueError:
		b=args.image_path.index("/", -12)+1
		cap_name=args.image_path[b:]
		#print(b)
		capturing= False
		captured = True
	else:
		print("Camera is not connected AND load images from memory is not requested")
		break
	## If alignment mode is ORB
	if args.alignment_mod == "ORB" and captured:
		if (abs(lastAltitude-altitude) > args.realignment_alt) or (args.image_path is not None):
			## read GPS data (altitude) to align again and create Homography
			if (args.image_path is not None):
				print("Aligning images...")
			else:
				print("The altitude changed more than %dm (or GPS warn is true), Do new alignment:" % args.realignment_alt) 
			bands, homographies = alignFiveBands(images, BAND_NAME.index(REF_BAND),cap_name)
			aligned = True
		else:
			## using the homography
			print("The altitude changes was bellow %dm, Use previous alignment data:" % args.realignment_alt)
			bands= alignAllByHomography(images, homographies, BAND_NAME.index(REF_BAND),cap_name)
			aligned = True
  
	## Crop aligned images
	if args.cropping_mod and aligned :
		if (args.image_path is not None):
			print("Cropping images...")
		maxCrop= images[BAND_NAME.index(REF_BAND)].shape[0]*MAX_CROP_PERCENT
		bandsCropped =cropAlignedFiveBands(bands, BAND_NAME.index(REF_BAND), maxCrop,cap_name)
		cropped = True
		if cap_name is not None and ((args.storage_mod == "ALL") or (args.storage_mod == "MIN")):
			cv2.imwrite(path+ cap_name +"_BGR.tif", cv2.merge([bandsCropped[0],bandsCropped[1],bandsCropped[2]]))

	if args.detection_mod == 1 and cropped:
		if (args.image_path is not None):
			print("Detecting in images...")
		blueNIR , greenNIR , redNIR , edgeNIR, detected = detectChanges(bandsCropped, MIN_CHANGE, MAX_CHANGE)
		if cap_name is not None and (args.storage_mod == "ALL"):
			cv2.imwrite(path+ cap_name +"_blueNIR.tif", blueNIR)
			cv2.imwrite(path+ cap_name +"_greenNIR.tif", greenNIR)
			cv2.imwrite(path+ cap_name +"_redNIR.tif", redNIR)
			cv2.imwrite(path+ cap_name +"_edgeNIR.tif", edgeNIR)
			cv2.imwrite(path+ cap_name +"_detected.tif", detected)
			cv2.imwrite(path+ cap_name +"_detectedBGR.tif", cv2.merge([(bandsCropped[0]-detected), (bandsCropped[1]-detected), (bandsCropped[3]-detected)]))
		elif  cap_name is not None and (args.storage_mod == "MIN"):
			cv2.imwrite(path+ cap_name +"_detectedBGR.tif", cv2.merge([(bandsCropped[0]-detected), (bandsCropped[1]-detected), (bandsCropped[3]-detected)]))
	

	if (resp is None) and (args.image_path is None):
		break

	if (args.image_path is not None):
		if args.alignment_mod != "NO":
			print("A set of images loaded, processed and the results has been saved.")
		else:
			print("A set of images loaded without any process!")
		break

	while ((datetime.now()-lastCaptureTime).total_seconds()) < args.timer_period :
		pass
	if sys.stdin in select.select([sys.stdin], [],[], 0)[0]:
			line = raw_input()
			print("You pressed Enter (Exit).")
			if (args.image_path is None) and (resp is not None):
				camera_powerdown_ready()
			break

