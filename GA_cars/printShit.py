__author__ = 'Colin'
from numbers import diff



def printShit():
    for i in range(1201):
        print i


dx = 0.1
y = [1,2,3,4,4,5,6]
dy = diff(y)/dx
print dy
array([ 10.,  10.,  10.,   0.,  10.,  10.])