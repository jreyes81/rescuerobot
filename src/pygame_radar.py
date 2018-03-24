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
from std_msgs.msg import Float32,Bool,Int8

from robot_stop import stop_button # "STOP" button function. Takes in 5 arguments
from arc_inc import arc_inc # Markers for Arc Increments
from linesections import linesections # Creates the cross sectional lines
from labels import labels # Creates "Object Detected", "Object Out of Range", "Servo Angle"
from range_to_string import range_to_string # Creates range integers to string as well as degrees
from text_object_black import text_object_black

class RadarDisplay():
    def __init__(self):

        # Initializing values
        self.sx = 700 # width of screen [Adjust width and length to fit your screen size]
        self.sy = 700 # length of screen
        self.count = 0
        self.real_obj_dist = 0
        self.stop = False # Value to make robot stop
        self.estimated_obj_dist = 0
        self.servo_pos = 0
        self.angle = 0
        self.pos = 2
        self.green = (0,200,0) # color
        self.red = (200,0,0) # color
        self.brightred = (255,0,0) # color
        self.brightblue = (142,229,238) # color
        self.rate = rospy.Rate(60) # 60 Hz used to publish
        self.scan_once_to_herc = 0
        self.scan_once_from_herc = []

        # Subscribers
        self.sensormsgs = rospy.Subscriber("HerculesUltrasound_Range",Range, self.distcallback) #Used to be Float32. [Float 32, callback]
        self.servopos = rospy.Subscriber("HerculesUltrasound_Position",Float32, self.poscallback)
        self.scan_once = rospy.Subscriber("scan_once_return",Int8, self.scancb)

        # Publisher
        self.cmd_stop = rospy.Publisher('cmd_stop',Bool, queue_size=10) # Publishes "True" or "False" state of stop button
        self.scan_once_send = rospy.Publisher("scan_once_to_whls",Int8, queue_size=10) # Publishes 0 or 1 for radar sweep scanning state.
        rospy.on_shutdown(self.close)
        self.plot()

    def scancb(self,data):
        self.scan_once_from_herc = data.data # data comes out as 1 or 2

    def distcallback(self,range): # Takes in message "range" as input. Data comes back in cm
        real_obj_dist = range.range
        estimated_obj_dist = (real_obj_dist/25)*2 # Scales down object distance to fit on display. Radius of 25 cm translates to 2 units. 100 cm translates to 8 units
        self.real_obj_dist = real_obj_dist
        self.estimated_obj_dist = estimated_obj_dist

    def poscallback(self,data):
        servo_position = data.data # 645 (or 646) is 180 degrees while 191 is 0 degrees
        self.servo_pos = servo_position

    def plot(self):
        #self.pub_stop_button_state()
        pygame.init() # Initializing pygame
        #pygame.time.delay(2000) # Waits 2 seconds to get in synch with servo

        # Adjusting window
        depth_bits = 32 # Number of bits to use for color.

        # Initializing screen for display. Creates a display surface.
        init_screen = pygame.display.set_mode((self.sx, self.sy), 0, depth_bits) # Initializing screen for display
        title_screen = pygame.display.set_caption('RGRR Display')
        screen = pygame.display.get_surface() # Returns a reference to the current display.

        # Initialize variables
        Rrx = [0] *512 # Creates an array of 512 zeros. 512 is number of points
        Rry = [0] *512 # Creates an array of 512 zeros

        while not rospy.is_shutdown():
          self.rate.sleep()

          for i in range(4096): # 4096 covers entire circle and is how many times radius line is drawn
            self.pub_stop_button_state() # Publishes state information about stop button. Continous "False" when it is not pressed and continous "True" when presses
            self.pub_scan_once_state() # Publishes 0 continous when radar is still scanning, Publishes 1 continous when its done
            self.angle = i * (2 * math.pi)/72 #  Line is incremented by 0.08 degrees (6.283 is 2 pi)
            if self.count == 4096:
                self.count = 0
                self.scan_once_to_herc = 1
                #self.scan_once_display = False
            self.count = self.count + 1

            if i%8==0:
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/2) - (self.sx/10), 1) # Outer circle
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/10), 1) #Inner circle. radius was sx/pos/5*1
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/5), 1) # 2nd Inner circle. radius was sx/pos/5*2
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/5) + (self.sx/10), 1) # 3rd Inner Circle. radius was sx/pos/5*3
               pygame.draw.line(screen, self.green, ((self.sx/10), self.sy/self.pos), ((self.sx - (self.sx/10)), self.sy/self.pos)) # Horizontal Line. Origianlly from (0, sy/pos) to (sx,sy/pos)
               #pygame.draw.line(screen, self.brightred, (sx/self.pos, 100), (sx/self.pos, 900)) # Vertical Line
               pygame.draw.line(screen, self.brightblue, (self.sx/self.pos, (self.sy/10)), (self.sx/self.pos, (self.sy/2))) # Vertical Line

               # Modules imported and plotted
               self.stop = stop_button(self.sx,self.sy,self.stop,self.brightred,self.red) # Stop Button Feature
               labels(self.sx,self.sy,self.real_obj_dist)
               arc_inc(self.sx,self.sy)
               range_to_string(self.sx,self.sy,self.real_obj_dist,self.servo_pos)
               linesections(self.sx,self.sy,self.green)

               #pygame.time.delay(2000) # Waits 2 seconds to get in synch with servo
               if (self.scan_once_to_herc == 1 and self.scan_once_from_herc == 1): # Radar sweeps once then stops
                   pygame.display.update()
                   pygame.time.wait(30) # Sleeps the gui for 30 milliseonds to share CPU. Share with ROS
                   # Could also use pygame.time.delay() instead of time.wait
                   screen.fill((0, 20, 0, 0))

                   for event in pygame.event.get():
                      if event.type == pygame.QUIT:
                          pygame.quit()
                          exit()

               if self.scan_once_to_herc == 0 or self.scan_once_from_herc == 2: # True only happens in the begging when we run the program
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
                   elif self.estimated_obj_dist > 8: # If object distance is greater than 100 cm. It gets plotted at origin. 8 is the radius of the big circle
                       self.estimated_obj_dist = 0

                   if 0 <= self.count <= 2048:
                       #dx = self.sx/2 - self.sx/2 * math.cos(math.radians(self.angle)) # Starts from left side
                       x = self.sx/2 - (self.sx/2.5)* math.cos(math.radians(self.angle))
                       #dy = self.sy/2 - self.sx/2 * math.sin(math.radians(self.angle)) # Starts from top side
                       y = self.sy/2 - (self.sy/2.5) * math.sin(math.radians(self.angle))
                       # anti aliasing line: To make line smooth
                       pygame.draw.aaline(screen, self.brightred, (self.sx/2, self.sy/2), (x, y),5) # Takes about 10 seconds to sweep 180 degrees. Used to be dx and dy

                       rx = int(self.sx/2 - 50 * self.estimated_obj_dist * math.cos(math.radians(self.angle))) # Starts plotting left
                       ry = int(self.sy/2 - 50 * self.estimated_obj_dist * math.sin(math.radians(self.angle))) # Starts Plotting on top
                       Rrx[i/8] = rx
                       Rry[i/8] = ry
                       pygame.display.update()
                       pygame.time.wait(30) # Sleeps the gui for 30 milliseonds to share CPU. Share with ROS
                       # Could also use pygame.time.delay() instead of time.wait
                       screen.fill((0, 20, 0, 0))

                   elif 2048 < self.count <= 4096: #(self.count>2048 | self.count <=4096):
                       #dx = self.sx/2 - self.sx/2 * math.cos(math.radians(self.angle)) # Starts from right side
                       x = self.sx/2 + (self.sx/2.5)* math.cos(math.radians(self.angle-180))
                       #dy = self.sy/2 + self.sx/2 * math.sin(math.radians(self.angle)) # Starts from top side
                       y = self.sy/2 + (self.sy/2.5) * math.sin(math.radians(self.angle))
                      # anti aliasing line: To make line smooth
                       pygame.draw.aaline(screen, self.brightred, (self.sx/2, self.sy/2), (x, y),5) # Takes about 10 seconds to sweep 180 degrees. Used to be dx and dy

                       rx = int(self.sx/2 - 50 * self.estimated_obj_dist * math.cos(math.radians(self.angle))) # Starts plotting right
                       ry = int(self.sy/2 + 50 * self.estimated_obj_dist * math.sin(math.radians(self.angle))) # Starts plotting top
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


    def pub_stop_button_state(self):
        self.cmd_stop.publish(self.stop)

    def pub_scan_once_state(self):
        self.scan_once_send.publish(self.scan_once_to_herc)

    def close(self):
        pygame.quit()
        exit()
