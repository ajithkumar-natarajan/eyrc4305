import cv2
import numpy as nu
import serial as pys



sum = 17
c1 = 0
c2 = 0
c3 = 0
img1 = cv2.imread('imag.jpg')
img2 = img1[87:128,108:388]
img3 = img1[285:367,108:388]


"""
#cv2.imshow("image",img3)
#cv2.waitKey(0)

arr = [[1 for i in range(7)]for j in range(2)]
obs = [[1 for i in range(7)]for j in range(2)]
temp_array = [1 for i in range(7)]




#Track - 1 : Conversion to Matrix form

print 'Track-1'

for x in range(7):
	temp = img2[0:41,x*40:(x+1)*40]	
	if(temp[20,20,1] < 100):
		temp_array[x] = 0
		c1 = c1+1	


print temp_array
print 'The number of cavities in Track-1 is ' + str(c1)

#Track - 2 : Conversion to Matrix form

print 'Track-2'

for x in range(2):
	for y in range(7):
		temp = img3[x*40:(x+1)*40,y*40:(y+1)*40]
		if(temp[20,20,1] < 100):
			arr[x][y] = 0
			c2 = c2 + 1
		if(arr[x][y] == 1):
			for k in range(0,40,5):
				if(x==0):
					if(temp[3,k,2] < 100):
						obs[x][y] = 0
						c3 = c3 + 1
						break
				else:
					if(temp[36,k,2] < 100):
						obs[x][y] = 0
						c3 = c3 + 1
						break

	
for x in range(2):
	print arr[x]

print 'The number of cavities in Track-2 is ' + str(c2)

for x in range(2):
	print obs[x]
	
print 'The number of obstacles in Track-2 is ' + str(c3)

"""

#Digits : Conversion to Matrix form


ans = 0
comp = 0


b1 = img1[40:80,515:555]
for x2 in range(0,10):
	imgpath = 'digit/'+str(x2)+'.jpg'
	img3 = cv2.imread(imgpath)
	res = cv2.matchTemplate(img3,b1,cv2.TM_CCOEFF)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
	if (max_val > comp):
		comp = max_val
		ans = x2

print ans
cv2.imshow("b1",b1)
cv2.waitKey(0)

ans = 0
comp = 0

b2 = img1[40:80,595:635]

for x2 in range(0,10):
	imgpath = 'digits/'+str(x2)+'.jpg'
	img3 = cv2.imread(imgpath)
	res = cv2.matchTemplate(img3,b2,cv2.TM_CCOEFF)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
	#print min_val,max_val,min_loc,max_loc
	if (max_val > comp):
		comp = max_val
		ans = x2

print ans
cv2.imshow("b1",b2)
cv2.waitKey(0)

ans = 0
comp = 0

b3 = img1[345:385,510:550]

for x2 in range(0,10):
	imgpath = 'digits/'+str(x2)+'.jpg'
	img3 = cv2.imread(imgpath)
	res = cv2.matchTemplate(img3,b3,cv2.TM_CCOEFF)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
	#print min_val,max_val,min_loc,max_loc
	if (max_val > comp):
		comp = max_val
		ans = x2

print ans
cv2.imshow("b1",b3)
cv2.waitKey(0)

ans = 0
comp = 0

b4 = img1[345:385,595:635]
for x2 in range(0,10):
	imgpath = 'digits/'+str(x2)+'.jpg'
	img3 = cv2.imread(imgpath)
	res = cv2.matchTemplate(img3,b4,cv2.TM_CCOEFF)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
	if (max_val > comp):
		comp = max_val
		ans = x2

print ans
cv2.imshow("b1",b4)
cv2.waitKey(0)





boulders = [3,4,9,4]


#Process Sum

