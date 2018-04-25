#!/usr/bin/env python
import math

center_of_circle = (500,500)
left_edge = (100,500)
cent_to_lefedge = (center_of_circle[0]-left_edge[0],center_of_circle[1]-left_edge[1])
newpoint_x = [0] * 17
newpoint_y = [0] * 17
newpoint = [0] * 17
newpoint = [0,0] * 17
print(newpoint)
n = range(20,360,20)
print(n)
for i in range(16):
        newpoint_x[i] = ( math.cos(math.radians(n[i])) * cent_to_lefedge[0] ) + ( math.sin(math.radians(n[i])) * cent_to_lefedge[1] )
        newpoint_y[i] = ( -1 *math.sin(math.radians(n[i])) * cent_to_lefedge[0] ) + ( math.cos(math.radians(n[i])) * cent_to_lefedge[1] )
