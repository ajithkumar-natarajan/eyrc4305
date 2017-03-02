import numpy
import copy

boul = [7,5,5,5]

c1 = 4
c2 = 3

ar1 = []
ar2 = []

boul_val1 = []
boul_val2 = []

sum = 15

def compute(i,arr,cnt,su):
	#print i,cnt,su
	if(i>=4):
		if(cnt==c1 and su==sum):
			global ar1
			ar1 = copy.deepcopy(arr)
		if(cnt==c2 and su==sum):
			global ar2
			ar2 = copy.deepcopy(arr)
			#print ar2
		return
	if(cnt==c1 and su==sum):
		ar1 = arr
	if(cnt==c2 and su==sum):
		ar2 = arr
	compute(i+1,arr,cnt,su)
	array = copy.deepcopy(arr)
	array.append(boul[i])
	cnt1 = cnt+1
	su1 = su + boul[i]
	compute(i+1,array,cnt1,su1)

arr = []
compute(0,arr,0,0)

m1 = [0 for x in range(4)]
m2 = [0 for x in range(4)]

for x in range(len(ar1)):
	for y in range(4):
  		if(ar1[x] == boul[y] and m1[y]==0):
  			boul_val1.append(chr(ord('a')+y))
  			m1[y] = 1
  			break
  			
for x in range(len(ar2)):
	for y in range(4):
  		if(boul[y]==ar2[x] and m2[y]==0):
  			boul_val2.append(chr(ord('a')+y))
  			m2[y] = 1
  			break

print boul_val1
print boul_val2