#!/usr/bin/env python
# pygame_radar.py takes in information from ultrasound sensor and graphs it
# to a radar display

# Original Source Code:
# https://www.youtube.com/redirect?q=http%3A%2F%2Farkouji.cocolog-nifty.com%2Fblog%2F2016%2F02%2Fraspberry-pi360.html&event=video_description&v=Hqkki-Jl4Y0&redir_token=Mpq6_On6NVQa4GJB1g0-_uQ2zn98MTUxNTYzNDM4MUAxNTE1NTQ3OTgx

# Modified By: Jeovanny Reyes
# Modified On: February 10, 2018

# Subscriber: "HerculesUltrasound_Range" message topic of float 32 data type
# Pubslisher: "" Hercules motor control to stop robot from moving

# Raytheon Radar Guided Rescue Robot
# Cal State LA Senior Design

import math
from sys import exit
import time
import pygame
import numpy as np

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

class RadarDisplay():
    def __init__(self):
        rospy.init_node("radar_display", anonymous=True)
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        # Initializing and instantiating values
        self.sx = 1000 # width of screen [Adjust width and length to fit your screen size]
        self.sy = 1000 # length of screen
        self.count = 0
        self.real_obj_dist = 0
	    self.stop = False # Value to make robot stop
        self.estimated_obj_dist = 0
        self.servo_pos = 0
        self.angle = 0
        self.pos = 2
        self.green = (0,200,0) # color
        self.red = (200,0,0) # color
        self.black = (0,0,0) # color
        self.brightred = (255,0,0)
        self.brightblue = (142,229,238)
        self.brightgreen = (0,255,0)
        self.smalltext = 0

        # Subscribers
        self.sensormsgs = rospy.Subscriber("HerculesUltrasound_Range",Range, self.distcallback) #Used to be Float32. [Float 32, callback]
        self.servopos = rospy.Subscriber("HerculesUltrasound_Position",Float32, self.poscallback)
        self.plot()

        rospy.spin()

    def distcallback(self,range): # Takes in message "range" as input. Data comes back in cm
        real_obj_dist = range.range
        estimated_obj_dist = (real_obj_dist/25)*2
        self.real_obj_dist = real_obj_dist
        self.estimated_obj_dist = estimated_obj_dist

    def poscallback(self,data):
        servo_position = data.data # 645 (or 646) is 180 degrees while 191 is 0 degrees
       # servo_position = (servo_position - 190) * (180/455)
        self.servo_pos = servo_position

    def text_object_black(self,text, font): # For black text
        self.textSurface = font.render(text, True, self.black)
        return self.textSurface, self.textSurface.get_rect()

    def text_object_green(self,text, font): # For green text
        self.textSurface = font.render(text, True, self.green)
        return self.textSurface, self.textSurface.get_rect()

    def text_object_red(self,text, font):
        self.textSurface = font.render(text, True, self.red)
        return self.textSurface, self.textSurface.get_rect()

    def arc_inc(self):
        x_pos = self.sx * 0.328
        y_pos = self.sy * 0.460
        x_length = self.sx * 0.150
        y_width = self.sy * 0.100
        self.smalltext = pygame.font.Font("freesansbold.ttf",15)

        textSurf1, textRect1 = self.text_object_green("25 cm", self.smalltext)
        textRect1.center = ( (x_pos +(x_length/2)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf1, textRect1)

        textSurf2, textRect2 = self.text_object_green("50 cm", self.smalltext)
        textRect2.center = ( (x_pos +(x_length/2) - (self.sx/10)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf2, textRect2)

        textSurf3, textRect3 = self.text_object_green("75 cm", self.smalltext)
        textRect3.center = ( (x_pos +(x_length/2) - (self.sx/5)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf3, textRect3)

        textSurf4, textRect4 = self.text_object_green("100 cm", self.smalltext)
        textRect4.center = ( (x_pos +(x_length/2) - (self.sx*0.300)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf4, textRect4)

        textSurf5, textRect5 = self.text_object_green("25 cm", self.smalltext)
        textRect5.center = ( (x_pos +(x_length/2) + (self.sx/5)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf5, textRect5)

        textSurf6, textRect6 = self.text_object_green("50 cm", self.smalltext)
        textRect6.center = ( (x_pos +(x_length/2) + (self.sx*0.300)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf6, textRect6)

        textSurf7, textRect7 = self.text_object_green("75 cm", self.smalltext)
        textRect7.center = ( (x_pos +(x_length/2) + (self.sx/2.5)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf7, textRect7)

        textSurf8, textRect8 = self.text_object_green("100 cm", self.smalltext)
        textRect8.center = ( (x_pos +(x_length/2) + (self.sx/2)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf8, textRect8) # cm and deg markers # cm markers ,e.g. 20 cm

    def labels(self): # "Object Distance","Object Detected" and "Object Out of Range" label
        x_pos = self.sx * 0.625
        y_pos = self.sy * 0.005
        x_length = self.sx * 0.150
        y_width = self.sy * 0.100
        self.smalltext = pygame.font.Font("freesansbold.ttf",30)
        textSurfstop, textRectstop = self.text_object_green("Object Distance:", self.smalltext)
        textRectstop.center = ( (x_pos +(x_length/2)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurfstop, textRectstop)

        if self.real_obj_dist < 100:
            textSurfstop1, textRectstop1 = self.text_object_green("Object Detected", self.smalltext)
            textRectstop1.center = ( (x_pos +(x_length/2) - (self.sx*0.300)), y_pos+(y_width/2))
            pygame.display.get_surface().blit(textSurfstop1, textRectstop1)
        else:
            textSurfstop1, textRectstop1 = self.text_object_red("Object Out of Range", self.smalltext)
            textRectstop1.center = ( (x_pos +(x_length/2) - (self.sx*0.300)), y_pos+(y_width/2))
            pygame.display.get_surface().blit(textSurfstop1, textRectstop1)

        textSurfstop2, textRectstop2 = self.text_object_green("Servo Angle: ", self.smalltext)
        textRectstop2.center = ( (x_pos +(x_length/2) - self.sx * 0.550), y_pos+(y_width/2) + 850)
        pygame.display.get_surface().blit(textSurfstop2, textRectstop2)

    def range_to_string(self): # Label for displaying range value
        x_pos = self.sx * 0.800
        y_pos = self.sy * 0.005
        x_length = self.sx * 0.150
        y_width = self.sy * 0.100
        str_dist = str(self.real_obj_dist) # converting range integer to string
        str_pos = str(self.servo_pos) # Converting angle to string
        self.smalltext = pygame.font.Font("freesansbold.ttf",30)
        textSurfstop, textRectstop = self.text_object_green(str_dist + "cm", self.smalltext)
        textRectstop.center = ( (x_pos +(x_length/2)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurfstop, textRectstop)

        textSurfstop1, textRectstop1 = self.text_object_green(str_pos + "degrees", self.smalltext)
        textRectstop1.center = ( (x_pos +(x_length/2) - (self.sx*0.540)), y_pos+(y_width/2) + (self.sy*0.850))
        pygame.display.get_surface().blit(textSurfstop1, textRectstop1)

    def linesections(self): # Creating nine sections and degree markers
        center_of_circle = (self.sx/2,self.sy/2)
        left_edge = ((self.sx/10),self.sx/2)
        cent_to_lefedge = (center_of_circle[0]-left_edge[0],center_of_circle[1]-left_edge[1])
        self.smalltext = pygame.font.Font("freesansbold.ttf",(self.sx/50) - (self.sx/200))
        newpoint_x = [0] * 17
        newpoint_y = [0] * 17
        newpoint = [0] * 17
        n = range(20,360,20) # Creates 17 integers. Used for degree markers

        for i in range(17):
                newpoint_x[i] = ( math.cos(math.radians(n[i])) * cent_to_lefedge[0] ) + ( math.sin(math.radians(n[i])) * cent_to_lefedge[1] )
                newpoint_y[i] = ( -1 *math.sin(math.radians(n[i])) * cent_to_lefedge[0] ) + ( math.cos(math.radians(n[i])) * cent_to_lefedge[1] )

        newpoint1 = (center_of_circle[0] - newpoint_x[0], newpoint_y[0] + center_of_circle[1])
        newpoint2 = (center_of_circle[0] - newpoint_x[1], newpoint_y[1] + center_of_circle[1])
        newpoint3 = (center_of_circle[0] - newpoint_x[2], newpoint_y[2] + center_of_circle[1])
        newpoint4 = (center_of_circle[0] - newpoint_x[3], newpoint_y[3] + center_of_circle[1])
        newpoint5 = (center_of_circle[0] - newpoint_x[4], newpoint_y[4] + center_of_circle[1])
        newpoint6 = (center_of_circle[0] - newpoint_x[5], newpoint_y[5] + center_of_circle[1])
        newpoint7 = (center_of_circle[0] - newpoint_x[6], newpoint_y[6] + center_of_circle[1])
        newpoint8 = (center_of_circle[0] - newpoint_x[7], newpoint_y[7] + center_of_circle[1])
        newpoint9 = (center_of_circle[0] - newpoint_x[8], newpoint_y[8] + center_of_circle[1])
        newpoint10 = (center_of_circle[0] - newpoint_x[9], newpoint_y[9] + center_of_circle[1])
        newpoint11 = (center_of_circle[0] - newpoint_x[10], newpoint_y[10] + center_of_circle[1])
        newpoint12 = (center_of_circle[0] - newpoint_x[11], newpoint_y[11] + center_of_circle[1])
        newpoint13 = (center_of_circle[0] - newpoint_x[12], newpoint_y[12] + center_of_circle[1])
        newpoint14 = (center_of_circle[0] - newpoint_x[13], newpoint_y[13] + center_of_circle[1])
        newpoint15 = (center_of_circle[0] - newpoint_x[14], newpoint_y[14] + center_of_circle[1])
        newpoint16 = (center_of_circle[0] - newpoint_x[15], newpoint_y[15] + center_of_circle[1])
        newpoint17 = (center_of_circle[0] - newpoint_x[16], newpoint_y[16] + center_of_circle[1])

        textSurf1, textRect1 = self.text_object_green("20 deg", self.smalltext)
        textRect1.center = ( newpoint1[0] - (self.sx * 0.025), newpoint1[1])
        pygame.display.get_surface().blit(textSurf1, textRect1)


        textSurf2, textRect2 = self.text_object_green("40 deg", self.smalltext)
        textRect2.center = ( newpoint2[0] - (self.sx*0.030), newpoint2[1])
        pygame.display.get_surface().blit(textSurf2, textRect2)

        textSurf3, textRect3 = self.text_object_green("60 deg", self.smalltext)
        textRect3.center = ( newpoint3[0] - (self.sx*0.025), newpoint3[1] - (self.sy* 0.005))
        pygame.display.get_surface().blit(textSurf3, textRect3)

        textSurf4, textRect4 = self.text_object_green("80 deg", self.smalltext)
        textRect4.center = ( newpoint4[0] - (self.sx*0.025), newpoint4[1] - (self.sy * 0.010))
        pygame.display.get_surface().blit(textSurf4, textRect4)

        textSurf5, textRect5 = self.text_object_green("100 deg", self.smalltext)
        textRect5.center = ( newpoint5[0] + (self.sx*0.025), newpoint5[1] - (self.sy*0.010))
        pygame.display.get_surface().blit(textSurf5, textRect5)

        textSurf6, textRect6 = self.text_object_green("120 deg", self.smalltext)
        textRect6.center = ( newpoint6[0] + (self.sx*0.025), newpoint6[1] - (self.sy*0.005))
        pygame.display.get_surface().blit(textSurf6, textRect6)

        textSurf7, textRect7 = self.text_object_green("140 deg", self.smalltext)
        textRect7.center = ( newpoint7[0] + (self.sx*0.030), newpoint7[1] - (self.sy*0.010))
        pygame.display.get_surface().blit(textSurf7, textRect7)

        textSurf8, textRect8 = self.text_object_green("160 deg", self.smalltext)
        textRect8.center = ( newpoint8[0] + (self.sx*0.025), newpoint8[1] )
        pygame.display.get_surface().blit(textSurf8, textRect8)

        textSurf9, textRect9 = self.text_object_green("90 deg", self.smalltext)
        textRect9.center = ( self.sx/2,(self.sx*0.085))
        pygame.display.get_surface().blit(textSurf9, textRect9)

        screen = pygame.display.get_surface()

        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint1,5) # Left Hand Side
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint2,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint3,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint4,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint5,5) # Right Hand Side
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint6,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint7,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint8,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint9,5) # Bottom Right Side
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint10,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint11,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint12,5) # Bottom Left Side
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint13,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint14,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint15,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint16,5)
        pygame.draw.aaline(screen, self.green, center_of_circle, newpoint17,5) # Creates 9 sectors above x axis

    def stop_button(self): # Function to stop robot from moving
        mouse = pygame.mouse.get_pos() # Movement of mouse.
        click = pygame.mouse.get_pressed() # For button clicked
        self.smalltext = pygame.font.Font("freesansbold.ttf",30)
        x_pos = self.sx * 0.050
        y_pos = self.sy * 0.035
        x_length = self.sx * 0.150
        y_width = self.sy * 0.050
        stop = self.stop # this is equal to False

        if x_pos+x_length > mouse[0] > x_pos and y_pos+y_width > mouse[1] > y_width: # Hovering over box
            pygame.draw.rect(pygame.display.get_surface(),self.brightred,(x_pos,y_pos,x_length,y_width))
            if click[0] == 1:
                print('Robot has stopped moving')
		      stop = True # Value to make robot stop
        else:
            pygame.draw.rect(pygame.display.get_surface(),self.red,(x_pos,y_pos,x_length,y_width))

        textSurf,textRect = self.text_object_black("STOP", self.smalltext) # Insert text on box
        textRect.center = ( (x_pos +(x_length/2)), y_pos+(y_width/2))
        pygame.display.get_surface().blit(textSurf, textRect)
    return stop

    def plot(self):
        pygame.init() # Initializing pygame
        pygame.time.delay(2000) # Waits 2 seconds to get in synch with servo

        # Adjusting window
        depth_bits = 32 # Number of bits to use for color.

        # Initializing screen for display. Creates a display surface.
        init_screen = pygame.display.set_mode((self.sx, self.sy), 0, depth_bits) # Initializing screen for display
        title = pygame.display.set_caption('RGRR Display')
        screen = pygame.display.get_surface() # Returns a reference to the current dusplay.

        # Loop until the user clicks the close button.
        done = False

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

               self.stop_button()
               self.labels()
               self.arc_inc()
               self.range_to_string()
               self.linesections()
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
               if (self.count<=2048 | self.count>=0):
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


if __name__ == '__main__':
    RadarDisplay()
