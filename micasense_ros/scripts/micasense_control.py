#!/usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from micasense_ros.msg import Multispectral
from cv_bridge import CvBridge, CvBridgeError

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

pub=rospy.Publisher('capture_data', Multispectral, queue_size=10)
rospy.init_node('micontrol', anonymous=False, disable_signals=False)

## Default timer period (second) = period between each capture (in addition to the image processing required time if it is active) >0.25
DEFAULT_TIMER = 0.5
## Default flag for considering the operating altitude (0 or 1) = if 0, do capturing in any altitude. If 1, consider altitude from ros topic (as higher priority) then DSL
DEFAULR_ALTITUDE_FLAG = 0
## Default operating altitude (meters) = start capturing only if it is above the operating altitude
DEFAULT_OA = 2
## Default operating altitude tolerance (meters)
DEFAULT_OATOLE = 1
## Default panel detection flag (0 or 1) = if 1, no capture will repurt until detecting the panel (first capture would be the panel), or until the panel_timer (second) is passed
DEFAULT_PANEL = 0
DEFAULT_PANEL_TIMER = 10
## Default possition accuracy needed (metters)
DEFAULT_P_ACC=30
## Default store mode on SD card (0 or 1) = if 1, store raw tift files in SD card
DEFAULT_STORE=1

BAND_NAME = ('Blue','Green','Red','NIR','RedEdge')

parser = argparse.ArgumentParser()

#-adr ADDRESS -t TIMER -oaflag ALTITUDE_FLAG -oa OPERATING_ALTITUDE -oatole OA_TOLERANCE -cpl CAPTURE_PANEL -f FORMAT
parser.add_argument("-adr", "--address", dest = "image_path", help="If provided, images in the address will be loaded and published (instead of the camera)")
parser.add_argument("-ti", "--timer", dest = "timer_period", help="Period between each captures (seconds). (default: %.2f)" % (DEFAULT_TIMER),type=float)
parser.add_argument("-oaflag", "--oaflag", dest = "altitude_flag", help="Consider altitude for capturing: 1(=True), 0(=False). (default: %d)" % (DEFAULR_ALTITUDE_FLAG),type=int, choices=range(2))
parser.add_argument("-oa", "--oalt", dest = "operating_alt", help="Aircraft operating altitude above ground (meters). (default: %d)" % (DEFAULT_OA), type=int)
parser.add_argument("-oatole", "--oatole",dest ="oalt_tolerance", help="Tolerance altitude (meters). (default: %d)" % (DEFAULT_OATOLE), type=int)
parser.add_argument("-cpl", "--capturepanel",dest = "detect_panel", help="Cpture calibration panel first: 1(=True), 0(=False). (default: %d)" % (DEFAULT_PANEL),type=int, choices=range(2))
parser.add_argument("-fo", "--format",dest = "format_mod", help="Format SD card before start: 1(=True), 0(=False), 2(=True, exit after formating). (default: 0)",type=int, choices=range(3))
parser.add_argument("-st", "--store",dest = "store_mod", help="Store raw TIFF files in SD card (if it is available): 1(=True), 0(=False). (default: %d)" % (DEFAULT_STORE),type=int, choices=range(2))

args = parser.parse_args()
total_captures=0
pubpan=None
GPS_rostopic_received= False
lastGPS_rostopic_time=datetime.now()
## Binary mask of 5 band 11111 = 31
enabled_bands_raw=31
## Binary mask of 5 band 00000 = 0
enabled_bands_jpeg=0

def check_connection():
	attempt=1
	while (attempt>0):
		try:
			print ("Checking the Camera connection...")
			resp = requests.get(host + "/status", timeout=3)
			return resp.json()
		except requests.exceptions.RequestException:
			attempt-=1
			print ("Camera is not connected...")

def capture_bands(resp, altitude, latitude, longitude):
	try:
		global total_captures
		global store_capture
		captured_bands=Multispectral()
		bridge=CvBridge()
		## Sending the capture command, getting the capture id
		capture_id = requests.post(host + "/capture" , json={"store_capture":store_capture } , timeout=3).json()["id"]
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
					capture_id = requests.post(host + "/capture", json={"store_capture":store_capture } , timeout=3).json()["id"]
					startTime = datetime.now()
				else:
					#camera_powerdown_ready()
					raise Exception ("Capturing is not possible.")
		ti=rospy.get_rostime()
		captured_bands.capture_id=str(capture_id)
		if resp["utc_time_valid"] == True:
			captured_bands.time_string=str(resp["utc_time"]) ## TODO: change string to time
		else:
			captured_bands.time_string="Warning" 
		if (altitude is not None) and (latitude is not None) and (longitude is not None):
			captured_bands.altitude= altitude
			captured_bands.latitude= latitude
			captured_bands.longitude= longitude
		else:
			captured_bands.altitude= None
			captured_bands.latitude= None
			captured_bands.longitude= None
		capture_data = requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()
		## Check the status of the SD card (file location/name will change)
		if (resp["sd_status"]=="Ok") and (store_capture==True):
			cap_name = str(capture_data["raw_storage_path"]["1"])
			a=0
			b=0
		else:
			cap_name = str(capture_data["raw_cache_path"]["1"])
			a=6
			b=0
		if cap_name != "None":
			cap_name=cap_name[7-a:len(cap_name)-(6-b)]
			captured_bands.file_name=cap_name
			cap_name=cap_name.replace("/","-")
			## Publish captured bands on ROS
			for imageIndex in range(1,6):
				imresp = requests.get(host + str(capture_data["raw_cache_path"][str(imageIndex)]), stream=True).raw
				npimg = np.asarray(bytearray(imresp.read()), dtype="uint8")
				#img = cv2.imdecode(npimg, cv2.IMREAD_COLORL)
				#img = cv2.imdecode(npimg, cv2.IMREAD_GRAYSCALE)
				img = cv2.imdecode(npimg, cv2.LOAD_IMAGE_UNCHANGED)
				#if (img[:,:,0] == img[:,:,1]).all:
				#	print("+++++++++++++++++++++++++++++++++")
				img=img[:,:,0]
				msg = bridge.cv2_to_imgmsg(img,"8UC1")
				#captured_bands.raw_np_bands.append(npimg)
				captured_bands.raw_bands.append(msg)
				#captured_bands.raw_bands.append(npimg)
				captured_bands.raw_bands[imageIndex-1].header.seq=total_captures
				captured_bands.raw_bands[imageIndex-1].header.stamp=ti
				captured_bands.raw_bands[imageIndex-1].header.frame_id=str(imageIndex)
				## Save captured images (in manifold)
				#with open(path + cap_name + "_%d.tif" % (imageIndex), 'wb') as f:
				#	for chunk in im.iter_content(10240):
				#		f.write(chunk)
				#print("........................")
				#rospy.loginfo(captured_bands.raw_bands[imageIndex-1].header.seq)
				#rospy.loginfo(captured_bands.capture_id)
				#rospy.loginfo(captured_bands.file_name)
				#rospy.loginfo(captured_bands.time_string)
				if rospy.is_shutdown():
					#camera_powerdown_ready()
					print("\nYou pressed Ctrl + C (ROS node stopped).")
					return None
			#print("===========================")
			pub.publish(captured_bands)
		else:
			cap_name = None
		return cap_name
	except requests.exceptions.RequestException:
		raise Exception ("Camera did not respond to the capture command!")
	except:
		print("Publish captured bands failed!")
		raise

def capture_panel(resp):
	try:
		global total_captures
		global store_capture
		global panel_captured
		captured_bands=Multispectral()
		bridge=CvBridge()
		## Sending the capture panel command, getting the capture id
		capture_id = requests.post(host + "/capture/detect_panel", json={"detect_panel":True, "store_capture":store_capture}, timeout=3).json()["id"]
		detect_panel_status = requests.get(host + "/detect_panel", timeout=3).json()["detect_panel"]
		#print(capture_id)
		startTime = datetime.now()
		panel_captured = False
		print("Searching for the panel in next %d seconds..." %(DEFAULT_PANEL_TIMER))
		## Waiting for capture complete status
		while (str(requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()["status"]) != "complete") and (detect_panel_status==True) :
			detect_panel_status = requests.get(host + "/detect_panel", timeout=3).json()["detect_panel"]
			tdelta = datetime.now() - startTime
			capture_id = requests.post(host + "/capture", json={"store_capture":store_capture}, timeout=3).json()["id"]
			if(tdelta.total_seconds() >= DEFAULT_PANEL_TIMER):
				confirmation = str(raw_input("Camera didn't detect the panel after %ds, do you want to searching for another cyle? (y/n) " % DEFAULT_PANEL_TIMER)).lower().strip()
				while (confirmation[:1] != 'y') and (confirmation[:1] != 'n'):
					confirmation = str(raw_input("\nInvalid input! do you want to searching for another cyle? (y/n) ")).lower().strip()
				if (confirmation[:1] == 'y'):
					print("Continue searching for the panel...")
					startTime = datetime.now()
				else:
					## Turn off the panel detection
					detect_panel_status = requests.post(host + "/detect_panel", json={"abort_detect_panel":True}, timeout=3).json()["detect_panel"]
					print("Panel detection was True, but camera did not detect the panel after %d seconds. Now it is %s." % (DEFAULT_PANEL_TIMER , str(detect_panel_status)) )
					panel_captured = False
		if (str(requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()["status"]) == "complete"):
			panel_captured = True
			print("Panel detected.")
			ti=rospy.get_rostime()
			captured_bands.capture_id=str(capture_id)
			if resp["utc_time_valid"] == True:
				captured_bands.time_string=str(resp["utc_time"]) ## TODO: change string to time
			else:
				captured_bands.time_string="Warning" 
			capture_data = requests.get(host + "/capture/%s" % (str(capture_id)), timeout=3).json()
			if resp["sd_status"]=="Ok" and (store_capture==True):
				cap_name = str(capture_data["raw_storage_path"]["1"])
				a=0
				b=0
			else:
				cap_name = str(capture_data["raw_cache_path"]["1"])
				a=6
				b=0
			if cap_name != "None":
				cap_name=cap_name[7-a:len(cap_name)-(6-b)]
				captured_bands.file_name=cap_name
				cap_name=cap_name.replace("/","-")
				## Publish captured bands on ROS
				for imageIndex in range(1,6):
					imresp = requests.get(host + str(capture_data["raw_cache_path"][str(imageIndex)]), stream=True).raw
					npimg = np.asarray(bytearray(imresp.read()), dtype="uint8")
					#img = cv2.imdecode(npimg, cv2.CV_LOAD_IMAGE_UNCHANGED)
					#img = cv2.imdecode(npimg, cv2.IMREAD_GRAYSCALE)
					img = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
					#if (img[:,:,0] == img[:,:,1]).all:
					#	print("+++++++++++++++++++++++++++++++++")
					img=img[:,:,0]
					msg = bridge.cv2_to_imgmsg(img,"8UC1")
					#captured_bands.raw_np_bands.append(npimg)
					captured_bands.raw_bands.append(msg)
					#captured_bands.raw_bands.append(npimg)
					captured_bands.raw_bands[imageIndex-1].header.seq=total_captures
					captured_bands.raw_bands[imageIndex-1].header.stamp=ti
					captured_bands.raw_bands[imageIndex-1].header.frame_id=str(imageIndex)
					## Save captured images (in manifold)
					#with open(path + cap_name + "panel_%d.tif" % (imageIndex), 'wb') as f:
					#	for chunk in im.iter_content(10240):
					#		f.write(chunk)
					#print("........................")
					#rospy.loginfo(captured_bands.raw_bands[imageIndex-1].header.seq)
					#rospy.loginfo(captured_bands.capture_id)
					#rospy.loginfo(captured_bands.file_name)
					#rospy.loginfo(captured_bands.time_string)
					if rospy.is_shutdown():
						#camera_powerdown_ready()
						print("\nYou pressed Ctrl + C (ROS node stopped).")
						return None
				#print("===========================")
			else:
				cap_name= None
				captured_bands=None
			return cap_name , captured_bands
		else:
			cap_name= None
			captured_bands=None
			return cap_name , captured_bands
	except requests.exceptions.RequestException:
		raise Exception ("Camera did not respond to the capture panel command!")
	except:
		print("Publish captured panel failed!")
		raise

def loadFiveBand(imgPath):
	## imgPath = image address in memory contating its name without _{}.tif
	## Read all bands
	try:
		global total_captures
		captured_bands=Multispectral()
		bridge=CvBridge()
		ti=rospy.get_rostime()
		b= imgPath.index("/", -10)+1
		cap_name=imgPath[b:]
		captured_bands.capture_id=cap_name
		captured_bands.file_name=imgPath
		captured_bands.time_string="Memory"
		for imageIndex in range(1,6):
			imFilename = imgPath+"_{}.tif".format(imageIndex)
			im = cv2.imread(imFilename)
			im=im[:,:,0]
			msg = bridge.cv2_to_imgmsg(im,"8UC1")
			captured_bands.raw_bands.append(msg)
			captured_bands.raw_bands[imageIndex-1].header.seq=total_captures
			captured_bands.raw_bands[imageIndex-1].header.stamp=ti
			captured_bands.raw_bands[imageIndex-1].header.frame_id=str(imageIndex)
			print("........................")
			rospy.loginfo(captured_bands.raw_bands[imageIndex-1].header)
			rospy.loginfo(captured_bands.capture_id)
			rospy.loginfo(captured_bands.file_name)
			rospy.loginfo(captured_bands.time_string)
		print("===========================")
		pub.publish(captured_bands)

	except ValueError:
		raise Exception ("ValueError! Can not load images")
	#	b=imgPath.index("/", -12)+1
	#	cap_name=imgPath[b:]
	#	#print(b)
	except:
		raise Exception ("Can not load images")

def check_inputs():
	try:
		if (args.format_mod is not None):
			if (args.format_mod==2) and ((args.image_path is not None) or (args.timer_period is not None) or (args.altitude_flag is not None) or (args.operating_alt is not None) or (args.oalt_tolerance is not None)):
				raise Exception ("Format flag mode is 3 (format the SD card and exit requested), however, other parameters are also provided!")
		else:
			args.format_mod=0

		if (args.image_path is not None) and ((args.timer_period is not None) or (args.altitude_flag is not None) or (args.operating_alt is not None) or (args.oalt_tolerance is not None)):
			raise Exception ("The Address option is provided alongside other capturing options. Capturing is possible only either from the memory OR the camera, not both.")

		if (args.timer_period is not None) and (args.timer_period<0.25):
			raise Exception ("The timer period should be a positive value (>0.25), %.2f is not acceptable." % args.timer_period)
		elif (args.timer_period is None):
			args.timer_period=float(DEFAULT_TIMER)

		if (args.altitude_flag is not None):
			if args.altitude_flag == 0:
				if (args.operating_alt is not None) or (args.oalt_tolerance is not None):
					raise Exception ("Altitude flag is 0, but altitude operation parameters provided!")
		else:
			if (args.operating_alt is not None) or (args.oalt_tolerance is not None):
				args.altitude_flag = 1
			else:
				args.altitude_flag = DEFAULR_ALTITUDE_FLAG

		if (args.operating_alt is not None) and (args.operating_alt<0):
			raise Exception ("The operating altitude tolerance should be a positive value, %d is not acceptable." % (args.operating_alt))
		elif (args.operating_alt is None):
			args.operating_alt=DEFAULT_OA

		if (args.oalt_tolerance is not None):
			if (args.oalt_tolerance>=args.operating_alt):
				raise Exception ("The operating altitude tolerance should be less than the operating altitude (%d), %d is not acceptable." % (args.operating_alt, args.oalt_tolerance))
			if (args.oalt_tolerance<0): 
				raise Exception ("The operating altitude tolerance should be a positive value, %d is not acceptable." % (args.oalt_tolerance))
		elif (args.oalt_tolerance is None):
			args.oalt_tolerance=DEFAULT_OATOLE
	
		if (args.detect_panel is None):
			args.detect_panel= DEFAULT_PANEL

		if (args.store_mod is None):
			args.store_mod=DEFAULT_STORE
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

def reformat_sd_card(resp):
	try:
		reformat_status = "fail"
		print("\nFree space in the SD card is %s (GB)" % str(resp["sd_gb_free"]))
		confirmation = str(raw_input("Are you sure to reformat the SD card before start? (y/n) ")).lower().strip()
		while (confirmation[:1] != 'y') and (confirmation[:1] != 'n'):
			confirmation = str(raw_input("\nInvalid input! Are you sure to reformat the SD card before start? (y/n) ")).lower().strip()
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
	#try:
		lastAltitude=None
		altitude=None
		latitude=None
		longitude=None
		captured_panel=Multispectral()
		lastCaptureTime=datetime.now()
		lastCheckedTime=datetime.now()
		lastPrintTime=datetime.now()
		capturing = False
		panel_captured = False
		GPS_source=None
		capture_panel_implimented = False
		start_time=datetime.now()
		resp=None
		configured=False
		format_asked=False
		store_capture=[False,True][args.store_mod]

		## Reformat SD card if requested
		if args.format_mod!=0:
			resp=check_connection()
			if (resp is not None):
				if resp["sd_status"] != "NotPresent":
					reformat_sd_card(resp)
					format_asked=True
				else:
					store_capture=False
					print ("Reformat was requested, but SD card is not present (or it is unmounted)!")
			
		while (args.format_mod!=2):
			if not ((resp is None) and (args.image_path is None)):
				os.system('cls' if os.name== 'nt' else 'clear')
			cap_name = None
			pan_name = None
			captured = False
			period_time=args.timer_period
			if (args.image_path is None):
				## check connection
				resp=check_connection()		
				## Capture new images by camera (Ethernet) and publish them on ROS topic
				if (resp is not None):
					if configured==False:
						config_status = requests.post(host + "/config", json={"operating_alt":args.operating_alt , "operating_alt_tolerance":args.oalt_tolerance , "auto_cap_mode":"timer" , "timer_period":args.timer_period , "enabled_bands_jpeg":enabled_bands_jpeg , "enabled_bands_raw":enabled_bands_raw , "raw_format":"TIFF" , "streaming_allowed":"False" , "streaming_enable":"False"}, timeout=5).json()
						if args.altitude_flag==1:
							print("operating_alt:" + str(config_status["operating_alt"]))
							print("operating_alt_tolerance: " + str(config_status["operating_alt_tolerance"]))
						print("auto_cap_mode: " + str(config_status["auto_cap_mode"]))
						print("timer_period: " + str(config_status["timer_period"]))
						print("enabled_bands_jpeg: " + str(config_status["enabled_bands_jpeg"]))
						print("enabled_bands_raw: "+ str(config_status["enabled_bands_raw"]))
						print("raw_format: " + str(config_status["raw_format"]))
						print("streaming_allowed: " + str(config_status["streaming_allowed"]))
						print("streaming_enable: " + str(config_status["streaming_enable"]))
						print("store_capture: " + ["False","True"][args.store_mod])
						raw_input("Press any key to start!")
						configured=True

					print("SD:{:.2f}GB, Storage:{}, Altitude Flag:{}, GPS warn:{}, Panel Detection:{}, Panel detected:{}".format(
						resp["sd_gb_free"],
						str(store_capture),
						["False","True"][args.altitude_flag],
						str(resp["gps_warn"]),
						["False","True"][args.detect_panel],
						str(panel_captured)
						))
					print("Start time: ",start_time)
					print("Capture period: %.2fs" % period_time)
					print("Total captured sets: %d" % total_captures)
					if altitude is None:
						print("Current Altitude: Unknown")
					else:
						print("Current Altitude: %.2f" % altitude)
					if lastAltitude is None:
						print("Last Capture Altitude: Unknown")
					else:
						print("Last Capture Altitude: %.2f" % lastAltitude)
				
					## If SD card is full
					if ((resp["sd_status"]=="Full") or (resp["sd_warn"]==True)) and resp["sd_status"] != "NotPresent":
						## Check if the node has been started recently
						if (datetime.now()-start_time).total_seconds()<5 and format_asked==False:
							print("Reformatting the SD card is in progress...")
							reformat_sd_card(resp)
							format_asked=True
						else:
							store_capture=False
							#camera_powerdown_ready()
							print("SD card is full!")
					elif (resp["sd_status"]=="NotPresent"):
						store_capture=False
						print ("SD card is not present (or it is unmounted)!")

				
					## Capture panel if requested during the DEFAULT_PANEL_TIMER time
					if (args.detect_panel is not None) and (capture_panel_implimented== False):
						if args.detect_panel:
							pan_name , captured_panel = capture_panel(resp)
							capture_panel_implimented= True
					if pan_name is not None:
						if pubpan is None:
							pubpan=rospy.Publisher('panel_data', Multispectral, queue_size=10)
						pubpan.publish(captured_panel)

					
					if args.altitude_flag==1:
						#if GPS_rostopic_received==True:
						#	if (datetime.now()-lastGPS_rostopic_time).total_seconds()<=2:
						#		if (GPS_rostopic.p_acc<resp["p_acc"]):
						#			GPS_source="ROS"
						#			altitude=GPS_rostopic.altitude
						#			latitude=GPS_rostopic.latitude
						#			longitude=GPS_rostopic.longitude
						#		elif (resp["dls_status"]=="Ok") and (resp["gps_warn"]!=True) and (resp["p_acc"]<=DEFAULT_P_ACC):
						#			altitude=resp["alt_agl"]
						#			latitude=resp["gps_lat"]
						#			longitude=resp["gps_lon"]
						#			GPS_source="DLS"
						#		else: ## If positioning was not accurate
						#			GPS_source=None
						#			altitude=None
						#			latitude=None
						#			longitude=None
						#	else:
						#		GPS_rostopic_received=False

						if GPS_rostopic_received==False:	
							if (resp["dls_status"]!="Ok") or (resp["gps_warn"]==True) or (resp["p_acc"]>DEFAULT_P_ACC):
								altitude=None
								latitude=None
								longitude=None
								GPS_source=None
							else:
								altitude=resp["alt_agl"]
								latitude=resp["gps_lat"]
								longitude=resp["gps_lon"]
								GPS_source="DLS"
						
						if altitude is not None:
							## Checking the operating altitude for capturing
							if altitude >= (args.operating_alt + args.oalt_tolerance):
								capturing= True
							elif altitude <= (args.operating_alt - args.oalt_tolerance):
								capturing= False
						else:
							capturing= False
							period_time=3
							lastCheckedTime=datetime.now()
							print("DLS position accuarcy is: %fm" %resp["p_acc"])
							#print("GPS_ROS position accuarcy is: %fm" % GPS_rostopic.p_acc)
							print("Altitude is Unkown and Altitude flag is 1.")
							print("Waiting for", end=' ')
					else:
						capturing=True
				
					if capturing:
						## Capture if altitude_flag is 0 or the position is accurate and above the operating range
						cap_name = capture_bands(resp, altitude, latitude, longitude)
						if cap_name is not None:
							lastCaptureTime=datetime.now()
							total_captures+=1
							captured = True
						if altitude is not None:
							lastAltitude=altitude							

				else:
					print("Camera is not connected AND load images from memory is not requested")
					print("Waiting for the Camera... (or press Ctrl + C to exit)")
			
			else:
				#TODO: load several images
				print("Load images from memory is requested")
				try:
					loadFiveBand(args.image_path)
					lastCheckedTime=datetime.now()
					total_captures=1
					period_time=1.25
					capturing= False
					print("A set of images loaded, and published. Press Ctrl + C to exit.")
					print("Waiting for", end=' ')
				except:
					print ("Image address is not correct!")
					break

			while (capturing==True and (((datetime.now()-lastCaptureTime).total_seconds()) < period_time)) or (capturing==False and (((datetime.now()-lastCheckedTime).total_seconds()) <= period_time)):
				if capturing==False:
					if ((datetime.now()-lastPrintTime).total_seconds())>=1:
						print("%ds" %(period_time-(datetime.now()-lastCheckedTime).total_seconds()+1))
						lastPrintTime=datetime.now()
				if rospy.is_shutdown():
					break
			if rospy.is_shutdown():
				#camera_powerdown_ready()
				print("\nYou pressed Ctrl + C (ROS node stopped).")
				break
		rospy.signal_shutdown("You pressed Ctrl + C (ROS node stopped).")
		#camera_powerdown_ready()
		print("Shutting down")
	#except Exception:
		#camera_powerdown_ready()
	#	raise Exception
	#except:
		#camera_powerdown_ready()
	#	raise Exception("Error! Shutting down")
