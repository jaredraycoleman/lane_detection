import numpy as np
import cv2

def get_furthest_pixel(row, rightmost, start, end):
	step = 3
	
	row = row[int(start):int(end)]
	length = len(row)
	if rightmost:
		for i in range(0, length, step):
			if (row[length-i-1] == 255): return length-i-1 + (start-1)
	else:
		for i in range(0, length, step):
			if (row[i] == 255): return i + (start-1)
	
	return -1
	
def preprocess(img):
	gray = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY )
	kernel = np.ones((5,5),np.float32)/25
	gray = cv2.filter2D(gray,-1,kernel)
	ret, thresh = cv2.threshold(gray,185,255,cv2.THRESH_BINARY)	
	return thresh

def draw_lane(img, points, color):
	height, width, channels = img.shape
	#points = [x for x in points if x[1] > height * 0.2]
	n = len(points) - 1
	for i in range(n):
		j = i + 1
		if points[i][0] == 0 or points[j][0] == 0: continue
		cv2.line(img,(points[i][0], points[i][1]),(points[j][0], points[j][1]),color,5)
	return img
	
def poly(x, params):
	n = len(params) - 1
	res = 0
	for i in range(len(params)):
		res += params[n-i] * (x**i)
	return res
	
def fit_curve(points, degree):
	#z = np.array([i for i in points if i[1] < height * 0.80 and i[1] > height * 0.10])
	x = [i for i in points.T[1]]
	y = [i for i in points.T[0]]
	params = np.polyfit(x, y, degree)
	for point in points: point[0] = int(poly(point[1], params))
	return points
	
def filter_lane(current, previous, pfilter):
	if previous.size == 0: return current
	return (pfilter*previous + (1-pfilter)*current).astype(int)
		
def get_next_point(points, num):
	p = np.array([x for x in points if x[0] != 0])
	if p.size < 3: return -1
	
	x = [i for i in p.T[1]]
	y = [i for i in p.T[0]]
	params = np.polyfit(x, y, 1)
	
	return poly(num, params)

def get_lane_points(img):
	'''Pre-processing'''
	thresh = preprocess(img)
	
	'''Variables'''
	step = 10	#Step for row processing
	height, width, = thresh.shape
	stop = 0
	left = width * 0.2
	right = width * 0.8
	center = int(width/2)
	
	num_steps = int(height/step)
	lefts = np.zeros((num_steps,2), dtype=np.int)
	rights = np.zeros((num_steps,2), dtype=np.int)
	
	height -= 1
	while height > stop:
		num_steps -= 1
		
		l = get_furthest_pixel(thresh[height], True, 0, center-50)
		r = get_furthest_pixel(thresh[height], False, center+50, width-1)

		if l != -1 and r != -1: center = (r + l) / 2
		
		if l != -1: left = l
		else:  l = left
		
		if r != -1: right = r
		else:  r = right
		
		lefts[num_steps] = [int(l), height]
		rights[num_steps] = [int(r), height]
	
		height -= step

	return lefts, rights, thresh
	
def find_lanes(img, left_lane, right_lane):
	'''Variables'''
	pfilter = 0.90
	degree = 2
	
	'''Get unfiltered lane points'''
	l, r, thresh = get_lane_points(img)
	
	#img = draw_lane(img, l, (250, 0, 250))
	#img = draw_lane(img, r, (250, 0, 250))
	
	'''Fit curve'''
	l = fit_curve(l, degree)
	r = fit_curve(r, degree)
		
	'''Filter'''
	left_lane = filter_lane(l, left_lane, pfilter)
	right_lane = filter_lane(r, right_lane, pfilter)
	center_lane = np.array([[(left_lane[i][0] + right_lane[i][0])/2, left_lane[i][1]]for i, j in enumerate(left_lane)]).astype(int)
	
	'''Draw Lines'''
	white = np.full_like(img, 0)
	white = draw_lane(white, left_lane, (1, 1, 250))
	white = draw_lane(white, right_lane, (1, 1, 250))
	white = draw_lane(white, center_lane, (1, 250, 250))
		
	return white, left_lane, right_lane, thresh

'''
cap = cv2.VideoCapture('video2.mp4')
left_lane = np.array([])
right_lane = np.array([])
ret, frame = cap.read()
height, width, channels = frame.shape

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 30.0, (width, height))

while cap.isOpened():
	ret, frame = cap.read()
	if not ret: break
	
	frame, left_lane, right_lane = find_lanes(frame, left_lane, right_lane)
	cv2.imshow('frame', frame)
	out.write(frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
out.release()
cv2.destroyAllWindows()
	
'''
