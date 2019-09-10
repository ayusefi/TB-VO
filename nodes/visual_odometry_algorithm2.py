import cv2
import numpy as np
import matplotlib.pyplot as plt

im_count = 1

current_pos = np.zeros((3, 1))
current_rot = np.eye(3)

prev_image = None

lk_params = dict(winSize=(15,15),
		maxLevel = 2,
		criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_detector = cv2.FastFeatureDetector_create(threshold=25,
	nonmaxSuppression=True)

focal = 718.8560
principalPoint = (607.1928, 185.2157)

camera_matrix = np.array([[572.0, 0.0, 319.65],
	[0.0, 572.0, 240.16],
	[0.0, 0.0, 1.0]])

# create graph.
position_figure = plt.figure()
position_axes = position_figure.add_subplot(1, 1, 1)
error_figure = plt.figure()
rotation_error_axes = error_figure.add_subplot(1, 1, 1)
rotation_error_list = []
frame_index_list = []

position_axes.set_aspect('equal', adjustable='box')





def visualOdometry(image):

	global current_rot
	global current_pos
	global im_count
	global points0
	global keypoint
	global image1
	global position_figure
	global prev_image
	global lk_params
	global feature_detector
	global prev_keypoint
	global focal
	global principalPoint
	global position_axes



	# main process
	keypoint = feature_detector.detect(image, None)

	if prev_image is None:
		prev_image = image
		prev_keypoint = keypoint
		return

	points = np.array(map(lambda x: [x.pt], prev_keypoint),
		dtype=np.float32)

	p1, st, err = cv2.calcOpticalFlowPyrLK(prev_image,
		image, points,
		None, **lk_params)

	E, mask = cv2.findEssentialMat(p1, points,camera_matrix,
		cv2.RANSAC, 0.999, 1.0, None)

	points, R, t, mask = cv2.recoverPose(E, p1, points, camera_matrix)

	scale = 1.0


	current_pos += current_rot.dot(t) * scale
	current_rot = R.dot(current_rot)

	print current_pos[0][0], current_pos[2][0]





	position_axes.scatter(current_pos[0][0], current_pos[2][0])
	plt.pause(.01)

	img = cv2.drawKeypoints(image, keypoint, None)

	# cv2.imshow('image', image)
	cv2.imshow('feature', img)
	cv2.waitKey(1)

	prev_image = image
	prev_keypoint = keypoint
	




if __name__ == "__main__":
	visualOdometry(image)
