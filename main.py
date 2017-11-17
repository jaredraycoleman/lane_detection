import cv2
import numpy as np
import lanes

cap = cv2.VideoCapture('video.mp4')
left_lane = np.array([])
right_lane = np.array([])
ret, frame = cap.read()
height, width, channels = frame.shape

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 30.0, (width, height))
out_warped = cv2.VideoWriter('warped.avi',fourcc, 30.0, (width, height))
out_thresh = cv2.VideoWriter('thresh.avi',fourcc, 30.0, (width, height))

pts1 = np.float32([[width*0.44,height*0.20], 	#tl
				   [width*0.56,height*0.20],	#tr
				   [width*1.00,height*0.85],	#br
				   [width*0.00,height*0.85]])	#bl
				   
pts2 = np.float32([[width*0.20,height*0.00],	#tl
				   [width*0.80,height*0.00],	#tr
				   [width*0.80,height*1.00],	#br
				   [width*0.20,height*1.00]])	#bl
				   
draw_pts = pts1.astype(int).reshape((-1,1,2))
M = cv2.getPerspectiveTransform(pts1,pts2)
M_prime = cv2.getPerspectiveTransform(pts2,pts1)

while cap.isOpened():
	ret, frame = cap.read()
	if not ret: break
	
	img = np.copy(frame)
	#cv2.polylines(img,[draw_pts],True,(0,255,255))
	warped = cv2.warpPerspective(img,M,(width,height))
	warped, left_lane, right_lane, thresh = lanes.find_lanes(warped, left_lane, right_lane)
	unwarped = cv2.warpPerspective(warped,M_prime,(width,height))
	
	frame = np.where(unwarped == 0, frame, unwarped)
	
	cv2.imshow('original', frame)
	cv2.imshow('warped', warped)
	cv2.imshow('thresh', thresh)
	print(type(thresh))
	out.write(frame)
	out_warped.write(warped)
	out_thresh.write(thresh)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
out.release()
out_warped.release()
out_thresh.release()
cv2.destroyAllWindows()

'''
warped = cv2.warpPerspective(img,M,(width,height))

warped, l, r = lanes.find_lanes(warped, np.array([]), np.array([]))

unwarped = cv2.warpPerspective(warped,M_prime,(width,height))

cv2.imshow('original',img)
cv2.imshow('warped',warped)
cv2.imshow('unwarped',unwarped)

cv2.waitKey(0)
'''
