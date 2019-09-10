#!/usr/bin/env python

import cv2
import numpy as np
import os

from visual_odometry_algorithm2 import *
from PIL import Image
import glob


class TrtlbtVO:

	def __init__(self):

		image_format_left = '{:010d}.png'


		for im in range(232):


			image = cv2.imread(os.path.join('dataset/', image_format_left).format(im))

			visualOdometry(image)



def main():

	#Initialize Class
	clsstart = TrtlbtVO()


if __name__ == '__main__':
	main()
