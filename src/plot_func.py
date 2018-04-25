#!/usr/bin/env python
# Plot code: This function creates the radar display that is then called in pygame_radar.py
#
# Original Source Code:
# https://www.youtube.com/redirect?q=http%3A%2F%2Farkouji.cocolog-nifty.com%2Fblog%2F2016%2F02%2Fraspberry-pi360.html&event=video_description&v=Hqkki-Jl4Y0&redir_token=Mpq6_On6NVQa4GJB1g0-_uQ2zn98MTUxNTYzNDM4MUAxNTE1NTQ3OTgx
#
# Modified By: Jeovanny Reyes
# Modified On: February 10, 2018
#
# Raytheon Radar Guided Rescue Robot
# Cal State LA Senior Design

import math
from sys import exit
import time
import pygame
import numpy as np

#import rospy
#from sensor_msgs.msg import Range
#from std_msgs.msg import Float32

from robot_stop import stop_button # "STOP" button function. Takes in 5 arguments
from arc_inc import arc_inc # Markers for Arc Increments
from linesections import linesections # Creates the cross sectional lines
from labels import labels # Creates "Object Detected", "Object Out of Range", "Servo Angle"
from range_to_string import range_to_string # Creates range integers to string as well as degrees

def plot_func(xpos,ypos,real_obj_dist,estimated_obj_dist,servopos):
    pos = 2
    count = 0
    angle = 0
    stop = False
    brightred = (255,0,0) # color
    red = (200,0,0) # color
    green = (0,200,0) # color
    brightblue = (142,229,238) # color

    pygame.init() # Initializing pygame
    pygame.time.delay(2000) # Waits 2 seconds to get in synch with servo

    # Adjusting window
    depth_bits = 32 # Number of bits to use for color.

    # Initializing screen for display. Creates a display surface.
    init_screen = pygame.display.set_mode((xpos, ypos), 0, depth_bits) # Initializing screen for display
    title = pygame.display.set_caption('RGRR Display')
    screen = pygame.display.get_surface() # Returns a reference to the current dusplay.

    # Loop until the user clicks the close button.
    done = False

    # Initialize variables
    Rrx = [0] *512 # Creates an array of 512 zeros. 512 is number of points
    Rry = [0] *512 # Creates an array of 512 zeros

    while (True):
      if count == 4096:
          count = 0
      count = count + 1

      for i in range(4096): # 4096 covers entire circle and is how many times radius line is drawn
        angle = i * (2 * math.pi)/72 #  Line is incremented by 5 degrees (6.283 is 2 pi)

        if i%8==0:
           # Radius is 1000*(4/5) = 800 cm [400 cm]
           pygame.draw.circle(screen, green, (xpos/pos, ypos/pos), (xpos/2) - (xpos/10), 1) # Outer circle
           #Radius is 1000/5 = 200 cm [100 cm] [100 cm translates to distance of 2
           pygame.draw.circle(screen, green, (xpos/pos, ypos/pos), (xpos/10), 1) #Inner circle. radius was sx/pos/5*1
           # Radius is 1000*(2/5) = 400 cm [200 cm] [200 transaltes to distance of 3]
           pygame.draw.circle(screen, green, (xpos/pos, ypos/pos), (xpos/5), 1) # 2nd Inner circle. radius was sx/pos/5*2
           # Radius is 1000*(3/5) = 600 cm [300 cm]
           pygame.draw.circle(screen, green, (xpos/pos, ypos/pos), (xpos/5) + (xpos/10), 1) # 3rd Inner Circle. radius was sx/pos/5*3
           pygame.draw.line(screen, green, ((xpos/10), ypos/pos), ((xpos - (xpos/10)), ypos/pos)) # Horizontal Line. Origianlly from (0, sy/pos) to (sx,sy/pos)
           #pygame.draw.line(screen, self.brightred, (sx/self.pos, 100), (sx/self.pos, 900)) # Vertical Line
           pygame.draw.line(screen, brightblue, (xpos/pos, (ypos/10)), (xpos/pos, (ypos/2))) # Vertical Line

           stop_button(xpos,ypos,stop,brightred,red)#self.stop_button()
           labels(xpos,ypos,real_obj_dist)#self.labels()
           arc_inc(xpos,ypos)#self.arc_inc()
           range_to_string(xpos,ypos,real_obj_dist,servopos)#self.range_to_string()
           linesections(xpos,ypos,green)#self.linesections()
           #pygame.time.delay(2000) # Waits 2 seconds to get in synch with servo

           for j in range(512): # 512 is the number of points drawn for entire circle.
               deg = j * 5.625 / 8 # Increments by 40 degrees
               radar_deg = deg - angle # For first iteration we have 40 - 5 = 35 degrees
               if radar_deg <=0 :
                  col = int(255*((360+radar_deg)/360)**1.3) # col is color
                  pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5)
               else:
                  col = int(255*(radar_deg/360)**1.3)
                  pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5)

           if estimated_obj_dist < 0: # If object distance is smaller than 100 cm
               estimated_obj_dist = math.fabs(estimated_obj_dist) # Returns the absolute value of distance
               pygame.display.get_surface().blit(textSurfstop, textRectstop)
           elif estimated_obj_dist > 8: # If object distance is greater than 1oo cm. It gets plotted at origin
               estimated_obj_dist = 0

        #    dx = self.sx/2 - self.sx/2 * math.cos(math.radians(self.angle)) # Starts from lest side
        #    dy = self.sy/2 - self.sx/2 * math.sin(math.radians(self.angle)) # Starts from top side
        #    # anti aliasing line: To make line smooth
        #    pygame.draw.aaline(screen, self.brightred, (self.sx/2, self.sy/2), (dx, dy),5) # Takes about 10 seconds to sweep 180 degrees
           #
        #    rx = int(self.sx/2 - 50 * self.estimated_obj_dist * math.cos(math.radians(self.angle)))
        #    ry = int(self.sy/2 - 50 * self.estimated_obj_dist * math.sin(math.radians(self.angle)))
           #
        #    Rrx[i/8] = rx
        #    Rry[i/8] = ry
           #
        #    pygame.display.update()
        #    pygame.time.wait(30) # Sleeps the gui for 30 milliseonds to share CPU. Share with ROS
        #    # Could also use pygame.time.delay() instead of time.wait
        #    screen.fill((0, 20, 0, 0))
           if (count<=2048 | count>=0):
               dx = xpos/2 - xpos/2 * math.cos(math.radians(angle)) # Starts from left side
               dy = ypos/2 - xpos/2 * math.sin(math.radians(angle)) # Starts from top side
               # anti aliasing line: To make line smooth
               pygame.draw.aaline(screen, brightred, (xpos/2, ypos/2), (dx, dy),5) # Takes about 10 seconds to sweep 180 degrees

               rx = int(xpos/2 - 50 * estimated_obj_dist * math.cos(math.radians(angle)))
               ry = int(ypos/2 - 50 * estimated_obj_dist * math.sin(math.radians(angle)))
               Rrx[i/8] = rx
               Rry[i/8] = ry
               pygame.display.update()
               pygame.time.wait(30) # Sleeps the gui for 30 milliseonds to share CPU. Share with ROS
               # Could also use pygame.time.delay() instead of time.wait
               screen.fill((0, 20, 0, 0))

           elif (count>2048 | count <=4096):
              dx = xpos/2 - xpos/2 * math.cos(math.radians(angle)) # Starts from right side
              dy = ypos/2 + xpos/2 * math.sin(math.radians(angle)) # Starts from top side
              # anti aliasing line: To make line smooth
              pygame.draw.aaline(screen, brightred, (xpos/2, ypos/2), (dx, dy),5) # Takes about 10 seconds to sweep 180 degrees

              rx = int(xpos/2 - 50 * estimated_obj_dist * math.cos(math.radians(angle)))
              ry = int(ypos/2 - 50 * estimated_obj_dist * math.sin(math.radians(angle)))
              Rrx[i/8] = rx
              Rry[i/8] = ry
              pygame.display.update()
              pygame.time.wait(30) # Sleeps the gui for 30 milliseonds to share CPU. Share with ROS
              # Could also use pygame.time.delay() instead of time.wait
              screen.fill((0, 20, 0, 0))

           for event in pygame.event.get():
               if event.type == pygame.QUIT:
                   pygame.quit()
                   exit()
