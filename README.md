# micacontrol
This repository contains the python script to control the [MicaSense RedEdge-MX](https://micasense.com/rededge-mx/) camera without the SkyPort kit but with the [DJI Manifold 2-G](https://www.dji.com/ca/manifold-2) to do some onboard pixel-based image processing. To communicate with and control the camera, the [HTTP API](http://micasense.github.io/rededge-api/api/http.html) has been used. To powerup the camera, a DC-DC converter or [Power Distribution Borad](https://www.amazon.ca/s?k=power+distribution+board+bec+6s&dc&ref=a9_sc_1) is needed to reduce the 24v of the DJI power distributor, or the PSDK connector, to the working range of the camera which is 4.2v to 15.8v.

To mount the camera to the [Upward Gimbal Connector](https://m.dji.com/ca/product/m300-upward-gimbal-connector) without using the [SkyPort kit](https://micasense.com/dji-skyport-kits/), and mounting the [DSL 2](https://micasense.com/shop/DLS-2-p121781107) sensor on the top of the Manifold, some 3D printed [components](https://github.com/Ehsan67m/micacontrol/blob/main/Mounting.zip) will be needed.

To connect the camera to the Manifold via the Ethernet connector, a custom [ethernet adapter cable](https://support.micasense.com/hc/en-us/articles/360044274294-How-to-create-an-ethernet-adapter-cable-for-RedEdge-M-MX) is needed.

## Requirements
OpenCV

To install OpenCV follow the instruction on its [website](https://docs.opencv.org/master/d2/de6/tutorial_py_setup_in_ubuntu.html). It will install the numpy as well, othrewise:
```bash
sudo apt-get update
sudo apt-get install python-numpy
```
rasterio
```bash
sudo apt-get install python-rasterio
```
argparse
```bash
sudo apt-get install python-argparse
```
requests
```bash
sudo apt-get install python-requests
```
natsort
```bash
sudo apt-get install python-natsort
```

Also installing the imutils for python 2 is required, to do so
```bash
sudo apt install python-pip
pip2 install --upgrade imutils
```

## Usage
Different options are considered for this code:
Address option: if provided, the alignment, cropping, and detection will be used for images in the address (instead of the camera). e.g. If the name of a set of files be like NAME_1.tif to NAME_5.tif, only the NAME of the file should be entered (without _#.tif)
```bash
-adr ADDRESS/NAME
--address ADDRESS/NAME
```
Timer option: Period between each captures (seconds).
```bash
-t VALUE
--timer VALUE
```
Operating altitude: Aircraft operating altitude above ground (meters). (Do capturing above this altitude)
```bash
-oa VALUE
--oalt VALUE
```
Operating altitude tolerance: Tolerance of the operating altitude (meters).
```bash
-oatole VALUE
--oatole VALUE
```
Detection mode option: To do the pixel-based detection (chanegs in bands) 1(=True), 0(=False)
```bash
-d VALUE
--detection VALUE
```
Alignment modes option: ORB, NO
```bash
-ali VALUE
--alignment VALUE
```
Realignment range option: Redo the alignment after changing the altitude for this amount (meters)
```bash
-reali VALUE
--realignment VALUE
```
Cropping option: To crop aligned bands, 1 (=True), 0(=False)
```bash
-cr VALUE
--crop VALUE
```
Print the processing states option: 1 (=True), 0(=False)
```bash
-p VALUE
--print VALUE
```
Capture panel option: If enabled, searching for the panel at the beginning of the process will be activated
```bash
-cpl VALUE
--capturepanel VALUE
```
Format SD card option: To format the SD card memory before start, 1 (=True), 0(=False)
```bash
--f VALUE
--format VALUE
```
Storage mode option: To store results in the Manifold memory: NO, MIN, ALL
```bash
-s VALUE
--storage VALUE
```

For example,
```bash
python2.7 control_micasense.py -t 0 -oa 3 -oatole 2 -d 0 -ali NO -cr 0 -f 0
```
Will start the capturing above 5 (3+2) m and will continue capturing every 0 seconds (without delay), until the altitude goes below 1 (3-2). The detection, alignment, and cropping will not be performed. The SD card will not reformat. However, due to the initial values (use option --h to know default values), the below line will also do the same job
```bash
python2.7 control_micasense.py -ali NO
```

NOTE: By pressing Enter key, the process will be finished and the "ready to power down" command will be sent to the camera. The camera will then unmount the SD card safely. Even by re-running the code, the SD card is still unmounted. To mount the SD card again the camera should be turned off and on again.

## ROS
The code is divided into 4 scripts to have the ROS package. The micasense_control.py is for communicating with the Micasense camera and publish the captured images on the ROS topic. The micasense_image_processor.py will use the published images to create a BGR image and do the pixel-based detection. Then it will publish results on two different topics. Each python script has a viewer, that shows images (resized images).
The below image, shows the structure of the ROS nodes and topics:
![alt text](https://github.com/Ehsan67m/micacontrol/blob/main/Screenshot.png?raw=true)
