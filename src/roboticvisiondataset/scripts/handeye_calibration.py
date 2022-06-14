#!/usr/bin/env python

# Need to import here as well for some reason
import cv2

import sys

from rqt_gui.main import Main


plugin = 'rqt_handeye_calibration.New_HandeyeCalibration' #rqt_handeye_calibration.HandeyeCalibration
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
