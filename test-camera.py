import cv2

from smart_landing.rosetta_camera import RosettaCamera

cam = RosettaCamera()
cam.connect()

windowName = "Video Test"
cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)

while True:
	img = cam.getImage()
	cv2.imshow(windowName, img)

	key = cv2.waitKey(1)
	if key == 27:
		break

	if cv2.getWindowProperty(windowName, cv2.WND_PROP_VISIBLE) < 1:
		break

cv2.destroyAllWindows()