# micacontrol
This repository contains the python script to control the [MicaSense RedEdge-MX](https://micasense.com/rededge-mx/) camera without the SkyPort kit but with the [DJI Manifold 2-G](https://www.dji.com/ca/manifold-2) to do some onboard pixel-based image processing.

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
