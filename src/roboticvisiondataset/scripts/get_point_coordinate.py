#!/usr/bin/env python

# Need to import here as well for some reason
import cv2

import sys

from rqt_gui.main import Main


plugin = 'point_coordinate.GetPointcoordinate'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
