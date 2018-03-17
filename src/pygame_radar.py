#!/usr/bin/env python
# pygame_radar.py takes in information from ultrasound sensor and graphs it
# to a radar display

# Original Source Code:
# https://www.youtube.com/redirect?q=http%3A%2F%2Farkouji.cocolog-nifty.com%2Fblog%2F2016%2F02%2Fraspberry-pi360.html&event=video_description&v=Hqkki-Jl4Y0&redir_token=Mpq6_On6NVQa4GJB1g0-_uQ2zn98MTUxNTYzNDM4MUAxNTE1NTQ3OTgx

# Modified By: Jeovanny Reyes
# Modified On: February 10, 2018

# Subscriber: "HerculesUltrasound_Range" message topic of float 32 data type
# Pubslisher: "cmd_stop" send information about when "STOP" button is pressed

# Raytheon Radar Guided Rescue Robot
# Cal State LA Senior Design

import math
from sys import exit
import time
import pygame
import numpy as np

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32,Bool

from robot_stop import stop_button # "STOP" button function. Takes in 5 arguments
from arc_inc import arc_inc # Markers for Arc Increments
from linesections import linesections # Creates the cross sectional lines
from labels import labels # Creates "Object Detected", "Object Out of Range", "Servo Angle"
from range_to_string import range_to_string # Creates range integers to string as well as degrees
#from plot_func import plot_func
from text_object_black import text_object_black

class RadarDisplay():
    def __init__(self):

        # Initializing values
        self.sx = 600 # width of screen [Adjust width and length to fit your screen size]
        self.sy = 600 # length of screen
        self.count = 0
        self.real_obj_dist = 0
        self.stop = False # Value to make robot stop
        self.estimated_obj_dist = 0
        self.servo_pos = 0
        self.angle = 0
        self.pos = 2
        self.green = (0,200,0) # color
        self.red = (200,0,0) # color
        self.brightred = (255,0,0)
        self.brightblue = (142,229,238)

        # Subscribers
        self.sensormsgs = rospy.Subscriber("HerculesUltrasound_Range",Range, self.distcallback) #Used to be Float32. [Float 32, callback]
        self.servopos = rospy.Subscriber("HerculesUltrasound_Position",Float32, self.poscallback)
        #self.plot()

        # Publisher
        self.cmd_stop = rospy.Publisher('cmd_stop',Bool, queue_size=10)
        #self.cmd_stop.publish(self.stop) # Publishes "True" or "False" value from when stop button is pressed

        self.stop = self.plot() # Returns boolean value for when "STOP" button is pressed

    def distcallback(self,range): # Takes in message "range" as input. Data comes back in cm
        real_obj_dist = range.range
        estimated_obj_dist = (real_obj_dist/25)*2
        self.real_obj_dist = real_obj_dist
        self.estimated_obj_dist = estimated_obj_dist

    def poscallback(self,data):
        servo_position = data.data # 645 (or 646) is 180 degrees while 191 is 0 degrees
        self.servo_pos = servo_position

    def plot(self):
        pygame.init() # Initializing pygame
        #pygame.time.delay(2000) # Waits 2 seconds to get in synch with servo

        # Adjusting window
        depth_bits = 32 # Number of bits to use for color.

        # Initializing screen for display. Creates a display surface.
        init_screen = pygame.display.set_mode((self.sx, self.sy), 0, depth_bits) # Initializing screen for display
        title = pygame.display.set_caption('RGRR Display')
        screen = pygame.display.get_surface() # Returns a reference to the current display.

        # Initialize variables
        Rrx = [0] *512 # Creates an array of 512 zeros. 512 is number of points
        Rry = [0] *512 # Creates an array of 512 zeros

        while (True):
          if self.count == 4096:
              self.count = 0
          self.count = self.count + 1

          for i in range(4096): # 4096 covers entire circle and is how many times radius line is drawn
            self.angle = i * (2 * math.pi)/72 #  Line is incremented by 5 degrees (6.283 is 2 pi)

            if i%8==0:
               # Radius is 1000*(4/5) = 800 cm [400 cm]
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/2) - (self.sx/10), 1) # Outer circle
               #Radius is 1000/5 = 200 cm [100 cm] [100 cm translates to distance of 2
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/10), 1) #Inner circle. radius was sx/pos/5*1
               # Radius is 1000*(2/5) = 400 cm [200 cm] [200 transaltes to distance of 3]
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/5), 1) # 2nd Inner circle. radius was sx/pos/5*2
               # Radius is 1000*(3/5) = 600 cm [300 cm]
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/5) + (self.sx/10), 1) # 3rd Inner Circle. radius was sx/pos/5*3
               pygame.draw.line(screen, self.green, ((self.sx/10), self.sy/self.pos), ((self.sx - (self.sx/10)), self.sy/self.pos)) # Horizontal Line. Origianlly from (0, sy/pos) to (sx,sy/pos)
               #pygame.draw.line(screen, self.brightred, (sx/self.pos, 100), (sx/self.pos, 900)) # Vertical Line
               pygame.draw.line(screen, self.brightblue, (self.sx/self.pos, (self.sy/10)), (self.sx/self.pos, (self.sy/2))) # Vertical Line

            #    smalltext = pygame.font.Font("freesansbold.ttf",30)
            #    xposlength, yposwidth, mouse, x_pos, y_pos, x_length, y_width = stop_button(self.sx,self.sy,self.stop,self.brightred,self.red)
            #    if xposlength >mouse[0] > x_pos and yposwidth > mouse[1] > y_width:
            #        pygame.draw.rect(pygame.display.get_surface(),self.brightred,(x_pos,y_pos,x_length,y_width))
            #    else:
            #        pygame.draw.rect(pygame.display.get_surface(),self.red,(x_pos,y_pos,x_length,y_width))
               #
            #    textSurf, textRect = text_object_black("STOP",smalltext)
            #    textRect.center = ((x_pos +(x_length/2)), y_pos+(y_width/2))
            #    pygame.display.get_surface().blit(textSurf, textRect)

               self.stop = stop_button(self.sx,self.sy,self.stop,self.brightred,self.red)

               labels(self.sx,self.sy,self.real_obj_dist)#self.labels()
               arc_inc(self.sx,self.sy)#self.arc_inc()
               range_to_string(self.sx,self.sy,self.real_obj_dist,self.servo_pos)#self.range_to_string()
               linesections(self.sx,self.sy,self.green)#self.linesections()
               #pygame.time.delay(2000) # Waits 2 seconds to get in synch with servo

               for j in range(512): # 512 is the number of points drawn for entire circle.
                   deg = j * 5.625 / 8 # Increments by 40 degrees
                   radar_deg = deg - self.angle # For first iteration we have 40 - 5 = 35 degrees
                   if radar_deg <=0 :
                      col = int(255*((360+radar_deg)/360)**1.3) # col is color
                      pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5)
                   else:
                      col = int(255*(radar_deg/360)**1.3)
                      pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5)

               if self.estimated_obj_dist < 0: # If object distance is smaller than 100 cm
                   self.estimated_obj_dist = math.fabs(self.estimated_obj_dist) # Returns the absolute value of distance
                   pygame.display.get_surface().blit(textSurfstop, textRectstop)
               elif self.estimated_obj_dist > 8: # If object distance is greater than 1oo cm. It gets plotted at origin
                   self.estimated_obj_dist = 0

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
               if 0 <= self.count <= 2048:
                   dx = self.sx/2 - self.sx/2 * math.cos(math.radians(self.angle)) # Starts from left side
                   dy = self.sy/2 - self.sx/2 * math.sin(math.radians(self.angle)) # Starts from top side
                   # anti aliasing line: To make line smooth
                   pygame.draw.aaline(screen, self.brightred, (self.sx/2, self.sy/2), (dx, dy),5) # Takes about 10 seconds to sweep 180 degrees

                   rx = int(self.sx/2 - 50 * self.estimated_obj_dist * math.cos(math.radians(self.angle)))
                   ry = int(self.sy/2 - 50 * self.estimated_obj_dist * math.sin(math.radians(self.angle)))
                   Rrx[i/8] = rx
                   Rry[i/8] = ry
                   pygame.display.update()
                   pygame.time.wait(30) # Sleeps the gui for 30 milliseonds to share CPU. Share with ROS
                   # Could also use pygame.time.delay() instead of time.wait
                   screen.fill((0, 20, 0, 0))

               elif (self.count>2048 | self.count <=4096):
                  dx = self.sx/2 - self.sx/2 * math.cos(math.radians(self.angle)) # Starts from right side
                  dy = self.sy/2 + self.sx/2 * math.sin(math.radians(self.angle)) # Starts from top side
                  # anti aliasing line: To make line smooth
                  pygame.draw.aaline(screen, self.brightred, (self.sx/2, self.sy/2), (dx, dy),5) # Takes about 10 seconds to sweep 180 degrees

                  rx = int(self.sx/2 - 50 * self.estimated_obj_dist * math.cos(math.radians(self.angle)))
                  ry = int(self.sy/2 - 50 * self.estimated_obj_dist * math.sin(math.radians(self.angle)))
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
        return self.stop

    def pub_stop_button_state(self):
        self.cmd_stop.publish(self.stop)
