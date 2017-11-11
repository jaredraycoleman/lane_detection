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
	ret, thresh = cv2.threshold(gray,180,255,cv2.THRESH_BINARY)
	
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
	z = np.array([i for i in points if i[1] < height * 0.85])
	x = [i for i in z.T[1]]
	y = [i for i in z.T[0]]
	params = np.polyfit(x, y, degree)
	for point in points: point = [int(poly(point[1], params)), point[1]]
	return points #np.array([[int(poly(x[1], params)), x[1]] for x in points])
	
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
	left = width * 0.1
	right = width * 0.9
	center = int(width/2)
	bcenter = int(width * 0.05)		#bias for removing center
	kcenter = 0.117924528			#proportional constant for removing center
	
	num_steps = int(height/step)
	lefts = np.zeros((num_steps,2), dtype=np.int)
	rights = np.zeros((num_steps,2), dtype=np.int)
	
	height -= 1
	while height > stop:
		num_steps -= 1
		
		c_thresh = 0#int(kcenter * height + bcenter)
		
		l = get_furthest_pixel(thresh[height], True, left - 50, left + 50)
		r = get_furthest_pixel(thresh[height], False, right - 50, right + 50)

		if l != -1 and r != -1: center = (r + l) / 2
		
		if l != -1: left = l
		else:  l = left
		
		if r != -1: right = r
		else:  r = right
		
		lefts[num_steps] = [int(l), height]
		rights[num_steps] = [int(r), height]
	
		height -= step

	return lefts, rights
	
def find_lanes(img, left_lane, right_lane):
	'''Variables'''
	pfilter = 0.9
	degree = 1
	
	'''Get unfiltered lane points'''
	l, r = get_lane_points(img)
	
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
	img = draw_lane(img, left_lane, (0, 0, 250))
	img = draw_lane(img, right_lane, (0, 0, 250))
	img = draw_lane(img, center_lane, (0, 250, 250))
		
	return img, left_lane, right_lane


'''------------Main Program------------'''
cap = cv2.VideoCapture('video2.mp4')
left_lane = np.array([])
right_lane = np.array([])
ret, frame = cap.read()
height, width, channels = frame.shape

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 30.0, (width, height))

'''Start'''
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
	
