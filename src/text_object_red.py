#!/usr/bin/env python
# Text Color code: This contains the function to make text red
#
# Created By: Jeovanny Reyes
# Created On: March 5, 2018
#
#
# Raytheon Radar Guided Rescue robot
# Cal State LA

import pygame
import sys

def text_object_red(text,font):
    color_red = (200,0,0)
    textSurface = font.render(text,True,color_red)

    return textSurface, textSurface.get_rect()
