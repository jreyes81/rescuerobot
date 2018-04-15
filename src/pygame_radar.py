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
from text_object_black import text_object_black

class RadarDisplay():
    def __init__(self):

        # Initializing values
        self.sx = 700 # width of screen [Adjust width and length to fit your screen size]
        self.sy = 700 # length of screen
        self.count = 0
        self.count_data = 0 # Used to determine number of data points from sensors
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
        self.sync_servo = False
        self.scan_once_from_herc = []

        # Subscribers
        self.sensormsgs = rospy.Subscriber("HerculesUltrasound_Range",Range, self.distcallback) #Used to be Float32. [Float 32, callback]
        self.servopos = rospy.Subscriber("HerculesUltrasound_Position",Float32, self.poscallback)
        self.scan_once = rospy.Subscriber("scan_once_return",Float32, self.scancb) # Data coming in as 1 or 2. 1 means radar sweeps. 2 means

        # Publisher
        self.cmd_stop = rospy.Publisher('cmd_stop',Bool, queue_size=10) # Publishes "True" or "False" state of stop button
        self.scan_once_send = rospy.Publisher("scan_once_to_whls",Float32, queue_size=10) # Publishes 0 or 1 for radar sweep scanning state.
        # 0 for sweeping and 1 for not sweeping.
        rospy.on_shutdown(self.close)
        self.plot()

    def scancb(self,data):
        self.scan_once_from_herc = data.data # data comes out as 1 or 2
        if self.scan_once_from_herc == 2: # Right when we recieve a 2 telling the radar to sweep again...
            self.scan_once_to_herc = 0 # A 0 gets publish to the nav code telling the robot not to move as it is sweeping and collecting

    def distcallback(self,range): # Takes in message "range" as input. Data comes back in cm
        real_obj_dist = range.range
        estimated_obj_dist = (real_obj_dist/25)*2 # Scales down object distance to fit on display. Radius of 25 cm translates to 2 units. 100 cm translates to 8 units
        self.real_obj_dist = real_obj_dist # Used to convert integer to string to put on display as numerical value
        self.estimated_obj_dist = estimated_obj_dist # Used to plot the radar point on display

    def poscallback(self,data):
        servo_position = data.data # 645 (or 646) is 180 degrees while 191 is 0 degrees
        self.servo_pos = servo_position

    def plot(self):
        #self.pub_stop_button_state()
        pygame.init() # Initializing pygame
        pygame.time.delay(3400) # Waits 2 seconds to get in synch with servo

        # Adjusting window
        depth_bits = 32 # Number of bits to use for color.

        # Initializing screen for display. Creates a display surface.
        init_screen = pygame.display.set_mode((self.sx, self.sy), 0, depth_bits) # Initializing screen for display
        title_screen = pygame.display.set_caption('RGRR Display')
        screen = pygame.display.get_surface() # Returns a reference to the current display.

        # Initialize variables
        Rrx = [0] *512 # Creates an array of 512 zeros. 512 is number of points. Ultrasound takes about 230 points worth of data when windshield sweeping
        Rry = [0] *512 # Creates an array of 512 zeros

        while not rospy.is_shutdown():
          self.rate.sleep()
          # 4096/72 = 56.8888888 ----
          for i in range(950): # 4096 (Used to be this value) and then 2048, now 1024. covers entire circle and is how many times radius line is drawn
            self.pub_stop_button_state() # Publishes state information about stop button. Continous "False" when it is not pressed and continous "True" when presses
            self.pub_scan_once_state() # Publishes 0 continous when radar is still scanning, Publishes 1 continous when its done
            #self.angle = i * (2 * math.pi)/72 #  Radar sweep line is incremented by 0.08 degrees (6.283 is 2 pi). Originally divided by 72
            self.angle = i * (360)/950 # This is in degrees: Used to be 360/2048 = .175, then it was 360/1024=.3516. Now its 360/1000=.36
            # If we divided by 52 it takes 8.6 seconds to sweep from 0 to 180 degrees. 42 takes 6.7. 32 takes 5.2. 22 takes 3.43
            if self.count == 950: # Use to be 4096, then 2048, now
                self.count = 0
                self.scan_once_to_herc = 1 # To make radar display stop sweeping
                #self.scan_once_display = False
            self.count = self.count + 1

            if i%8==0: # Used to be 8 pygame draws, now its 6print
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/2) - (self.sx/10), 1) # Outer circle
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/10), 1) #Inner circle. radius was sx/pos/5*1
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/5), 1) # 2nd Inner circle. radius was sx/pos/5*2
               pygame.draw.circle(screen, self.green, (self.sx/self.pos, self.sy/self.pos), (self.sx/5) + (self.sx/10), 1) # 3rd Inner Circle. radius was sx/pos/5*3
               pygame.draw.line(screen, self.green, ((self.sx/10), self.sy/self.pos), ((self.sx - (self.sx/10)), self.sy/self.pos)) # Horizontal Line. Origianlly from (0, sy/pos) to (sx,sy/pos)
               #pygame.draw.line(screen, self.brightred, (sx/self.pos, 100), (sx/self.pos, 900)) # Vertical Line
               pygame.draw.line(screen, self.brightblue, (self.sx/self.pos, (self.sy/10)), (self.sx/self.pos, (self.sy/2))) # Vertical Line
               #print ("Landed in i statement")
               # Modules imported and plotted
               self.stop = stop_button(self.sx,self.sy,self.stop,self.brightred,self.red) # Stop Button Feature
               labels(self.sx,self.sy,self.real_obj_dist)
               arc_inc(self.sx,self.sy)
               range_to_string(self.sx,self.sy,self.real_obj_dist,self.servo_pos)
               linesections(self.sx,self.sy,self.green)

               if (self.scan_once_to_herc == 1):
               #if (self.scan_once_to_herc == 1 and self.scan_once_from_herc == 1):
                   # scan_once_to_herc is 1 when we reach a count of 4096 that completes a full sweep in the display.
                   # scan_once_from_herc is always being sent as 1. as long as the navigation Code
                   # does not receive commands to make the robot move.
                   pygame.display.update()
                   pygame.time.wait(30) # Sleeps the gui for 30 milliseonds to share CPU. Share with ROS
                   # Could also use pygame.time.delay() instead of time.wait
                   screen.fill((0, 20, 0, 0))

                   for event in pygame.event.get():
                      if event.type == pygame.QUIT:
                          pygame.quit()
                          exit()

               if self.scan_once_to_herc == 0 or self.scan_once_from_herc == 2:
                   if self.sync_servo == False:
                       #pygame.time.delay(2500)
                       self.sync_servo = True
                   # 0 is initial value to have display sweep. 2 comes when hercules finishes moving
                #    for j in range(512): # 512 is the number of radar points drawn for entire circle. Used to be 512, then 256, now 512 again
                #        #deg = j * 5.625 / 8 # Increments by 0.703125 degrees. Used to be 5.625/8, then it was 4.21875/6, try 2.8125/4
                #        deg =  j * 5.625/8
                #        radar_deg = deg - self.angle # For first iteration we have 0.7 - 0.08 = 0.62 degrees. at j=206 we get 144.84 - 180
                #        if radar_deg <=0 :
                #           col = int(255*((360+radar_deg)/360)**1.6) # col is color. Color. Used to be **1.3. this one darkens
                #           pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5) # Plots previous point data
                #        else:
                #           col = int(255*(radar_deg/360)**1.3) # Color is light
                #           pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5) # Plots previous point data
                   #
                #    if self.estimated_obj_dist < 0: # If object distance is smaller than 100 cm
                #        self.estimated_obj_dist = math.fabs(self.estimated_obj_dist) # Returns the absolute value of distance
                #        pygame.display.get_surface().blit(textSurfstop, textRectstop)
                #    elif self.estimated_obj_dist > 8: # If object distance is greater than 100 cm. It gets plotted at origin. 8 is the radius of the big circle
                #        self.estimated_obj_dist = 0

                   if 0 <= self.count <= 475: #Used to be up to 1024, now 900
                       #dx = self.sx/2 - self.sx/2 * math.cos(math.radians(self.angle)) # Starts from left side
                       for j in range(512): # 512 is the number of radar points drawn for entire circle. Used to be 512, then 256, now 512 again
                           #deg = j * 5.625 / 8 # Increments by 0.703125 degrees. Used to be 5.625/8, then it was 4.21875/6, try 2.8125/4
                           deg =  j * 5.625/8
                           radar_deg = deg - self.angle # For first iteration we have 0.7 - 0.08 = 0.62 degrees. at j=206 we get 144.84 - 180
                           if radar_deg <=0 :
                              col = int(255*((360+radar_deg)/360)**1.3) # col is color. Color. Used to be **1.3. this one darkens
                              pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5) # Plots previous point data
                           else:
                              col = int(255*(radar_deg/360)**1.6) # Color is light
                              pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5) # Plots previous point data

                       if self.estimated_obj_dist < 0: # If object distance is smaller than 100 cm
                           self.estimated_obj_dist = math.fabs(self.estimated_obj_dist) # Returns the absolute value of distance
                           pygame.display.get_surface().blit(textSurfstop, textRectstop)
                       elif self.estimated_obj_dist > 8: # If object distance is greater than 100 cm. It gets plotted at origin. 8 is the radius of the big circle
                           self.estimated_obj_dist = 0
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

                   elif 474 < self.count <= 950: #Used to be from 1024 to 2048
                       #dx = self.sx/2 - self.sx/2 * math.cos(math.radians(self.angle)) # Starts from right side
                       for j in range(512): # 512 is the number of radar points drawn for entire circle. Used to be 512, then 256, now 512 again
                           #deg = j * 5.625 / 8 # Increments by 0.703125 degrees. Used to be 5.625/8, then it was 4.21875/6, try 2.8125/4
                           deg =  j * 5.625/8
                           radar_deg = deg - self.angle # For first iteration we have 0.7 - 0.08 = 0.62 degrees. at j=206 we get 144.84 - 180
                           if radar_deg <=0 :
                              col = int(255*(abs(radar_deg)/360)**1.6) # This one brightens
                              pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5) # Plots previous point data
                            #   col = int(255*((360-radar_deg)/360)**1.6) # col is color. Color. Used to be **1.3. this one darkens
                            #   pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5) # Plots previous point data
                           else:
                              col = int(255*((abs(radar_deg))/360)**1.6) # col is color. Color. Used to be **1.3. this one darkens
                              pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5) # Plots previous point data
                            #   col = int(255*(abs(radar_deg)/360)**1.6) # This one brightens
                            #   pygame.draw.circle(screen, (0,col,0),(Rrx[j-1],Rry[j-1]),5) # Plots previous point data

                       if self.estimated_obj_dist < 0: # If object distance is smaller than 100 cm
                           self.estimated_obj_dist = math.fabs(self.estimated_obj_dist) # Returns the absolute value of distance
                           pygame.display.get_surface().blit(textSurfstop, textRectstop)
                       elif self.estimated_obj_dist > 8: # If object distance is greater than 100 cm. It gets plotted at origin. 8 is the radius of the big circle
                           self.estimated_obj_dist = 0
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
            else:
                #print("Landed in el statement")
                time.sleep(0.001) # Used to sleep gui for a bit

    def pub_stop_button_state(self):
        self.cmd_stop.publish(self.stop)

    def pub_scan_once_state(self):
        self.scan_once_send.publish(self.scan_once_to_herc)

    def close(self):
        pygame.quit()
        exit()
