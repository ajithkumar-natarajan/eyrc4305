import cv2
import numpy as nu



sum = 17
c1 = 0
c2 = 0
c3 = 0
img1 = cv2.imread('image.jpg')
img2 = img1[116:157,110:390]
img3 = img1[315:397,110:390]
arr = [[1 for i in range(7)]for j in range(2)]
obs = [[1 for i in range(7)]for j in range(2)]



#Track - 1 : Conversion to Matrix form

print 'Track-1'
temp_array = [1 for i in range(7)]

for x in range(7):
	temp = img2[0:41,x*40:(x+1)*40]
	if(temp[20,20,0] < 200):
		temp_array[x] = 0
		c1 = c1+1	


print temp_array
print 'The number of cavities in Track-1 is ' + str(c1)


#Track - 2 : Conversion to Matrix form


print 'Track-2'

for x in range(2):
	for y in range(7):
		temp = img3[x*40:(x+1)*40,y*40:(y+1)*40]
		if(temp[20,20,0] < 200):
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

#Digits : Conversion to Matrix form


b1 = img1[80:120,500:540]
cv2.imshow("b1",b1)
cv2.waitKey(0)

b2 = img1[80:120,580:620]
cv2.imshow("b1",b2)
cv2.waitKey(0)

b3 = img1[357:397,510:550]
cv2.imshow("b1",b3)
cv2.waitKey(0)

b4 = img1[357:397,580:620]
cv2.imshow("b1",b4)
cv2.waitKey(0)






boulders = [3,4,9,4]


#Process Sum


