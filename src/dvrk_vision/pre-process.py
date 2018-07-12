import time
import itertools
import json

neighbors={}
path={}


def connect(pt1, pt2):
#	print("Connecting {} and {}".format(pt1,pt2))
	if pt1==pt2:
		print("Same Points")
		return []
	try:
		exists = path[(str(pt1,pt2))]
	except:
		if ( abs(pt1[0]-pt2[0])==1 and pt1[1]==pt2[1] ) or ( abs(pt1[1]-pt2[1])==1 and pt1[0]==pt2[0] ) or ( abs(pt1[0]-pt2[0])==1 and abs(pt1[1]-pt2[1])==1 ):
			path[str((pt1,pt2))]=[pt2]
			path[str((pt2,pt1))]=path[str((pt1,pt2))]
#			print("Nearest Neighbor")
			return [pt2]
		pt3 = list(pt2)
		for i in range(2):
			if pt1[i]>pt2[i]:
				pt3[i]+=1
			else:
				if pt1[i]<pt2[i]:
					pt3[i]-=1
#		print("Recurse")
		path[str((pt1,pt2))] = connect(pt1, tuple(pt3)) + [pt2]
		path[str((pt2,pt1))]=path[str((pt1,pt2))]
#		print(path[(pt1,pt2)])
		return path[str((pt1,pt2))]
#	print("Found existing Path")
	return exists

if __name__ == "__main__":
	size = 2500
	spread = 25
	limit = 400


	start = time.time()
	pixels = itertools.product(range(size),repeat=2)
	increments = itertools.product(range(-limit,limit),repeat=2)

	for p1,p2 in pixels:
		neigh=[]
		for i1,i2 in increments:
			if i1==0 and i2==0:
				continue
			if p1+i1<0 or p2+i2<0:
				continue
			if i1<=spread and i2<=spread:
				neigh.append((p1+i1, p2+i2))
			connect((p1,p2),(p1+i1,p2+i2))
		neighbors[str((p1,p2))]=neigh


	fneighbors = open("neighbors.json","w")
	fpath = open("path.json","w")
	json.dump(neighbors, fneighbors)
	json.dump(path, fpath)
#	json.dump({str(k):v for k,v in neighbors.items()}, fneighbors)
#	json.dump({str(k):v for k,v in path.items()}, fpath)
	end = time.time()
	print("Execution time: {}".format(end-start))