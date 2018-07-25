import time
import itertools
import cv2
import numpy as np
import random

size=1000
color1=[random.random(), random.random(), random.random()]
color1 =[0,1,0]
color2=[random.random(), random.random(), random.random()]
color2=[1,1,0]
spread=2

def connect(pt1, pt2, image):
	if pt1==pt2:
		return image
	slope = np.array(pt2)-np.array(pt1)

	if 0 in slope:
		image = line(pt1, pt2, image)
	else:
		image, pt3 = diag(pt1, pt2, image, slope)
		image = line(pt3, pt1, image)
		image = line(pt3, pt2, image)
	return image

def line(pt1, pt2, image):
	if pt1[0]!=pt2[0] and pt1[1]!=pt2[1]:
		return image
	slope = np.array(pt2)-np.array(pt1)
	xvals = [pt1[0], pt2[0]]
	yvals = [pt1[1], pt2[1]]
	minx = min(xvals)
	miny = min(yvals)
	maxx = max(xvals)
	maxy = max(yvals)
	abs_slope=abs(slope)
	if slope[0]==0:
		image[minx][miny:maxy] = np.ones((abs_slope[1], 1))
	else:
		image[minx:maxx, miny] = np.ones((abs_slope[0],3))
	return image

def diag(pt1, pt2, image, slope):
	xvals = [pt1[0], pt2[0]]
	yvals = [pt1[1], pt2[1]]
	minx = min(xvals)
	miny = min(yvals)
	maxx = max(xvals)
	maxy = max(yvals)
	gradient = slope[1]/float(slope[0])
	abs_slope=abs(slope)

	if list(abs_slope).index(min(abs_slope))==0:
		temp = np.eye(abs_slope[0])
		endpts = [(minx,miny),(maxx,miny+abs_slope[0])]
		if gradient<0:
			temp=np.rot90(temp)
			endpts = [(minx,miny+abs_slope[0]),(maxx,miny)]
		image[minx:maxx, miny:(miny+abs_slope[0]), 0] = temp
		image[minx:maxx, miny:(miny+abs_slope[0]), 1] = temp
		image[minx:maxx, miny:(miny+abs_slope[0]), 2] = temp
		if endpts[0] in [pt1,pt2]:
			mid = endpts[1]
		if endpts[1] in [pt1,pt2]:
			mid = endpts[0]
		return image, mid
	else:
		temp = np.eye(abs_slope[1])
		endpts = [(minx,miny),(minx+abs_slope[1],maxy)]
		if gradient<0:
			temp=np.rot90(temp)
			endpts = [(minx+abs_slope[1],miny),(minx,maxy)]
		image[minx:(minx+abs_slope[1]), miny:maxy, 0] = temp
		image[minx:(minx+abs_slope[1]), miny:maxy, 1] = temp
		image[minx:(minx+abs_slope[1]), miny:maxy, 2] = temp
		if endpts[0] in [pt1,pt2]:
			mid = endpts[1]
		if endpts[1] in [pt1,pt2]:
			mid = endpts[0]
		return image, mid

if __name__ == "__main__":
	for i in range(20):
		image = np.zeros((size,size,3))
		pt1 = (random.randint(0,size), random.randint(0,size))
		pt2 = (random.randint(0,size), random.randint(0,size))
		image = connect(pt1, pt2, image)
		image[pt1[0]-spread:pt1[0]+spread, pt1[1]-spread:pt1[1]+spread] = color1 
		image[pt2[0]-spread:pt2[0]+spread, pt2[1]-spread:pt2[1]+spread] = color2
		cv2.imshow("Image",image)
		cv2.waitKey(0)