#!/usr/bin/env python
# Text Color code: This contains the function to make text green
#
# Created By: Jeovanny Reyes
# Created On: March 5, 2018
#
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import pygame
import sys

def text_object_green(text,font):
    color_green = (0,200,0)
    textSurface = font.render(text,True,color_green)

    return textSurface, textSurface.get_rect()
