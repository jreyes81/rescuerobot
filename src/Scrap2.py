#!/usr/bin/env python
# Dictionaries

import Scrap1

    def nav_route(self, empty)
        if sector5.empty == 0:
            # opmode 1 conditions: object is only detected in sector 5
            # maneuver: robot goes forward
            if sector4.empty == 1 && sector6.empty == 1:
            return cases[opmode1]
            # opmode 2 conditions: object detected only in sector 5 and 4
            # maneuver: robot rotates RIGHT (diagonally), goes forward, rotates left (diagonally), goes forward 
            elif sector4.empty == 0 && sector6.empty == 1:
            return cases[opmode2]
            # opmode 3 conditions: object detected only in sector 5 and 6
            # maneuver: robot rotates left (diagonally), goes forward, rotates right (diagonally), goes forward
            elif sector5.empty == 0 && sector4.empty == 0 && sector6.empty == 1:
            return cases[opmode3]
        # stopmode is the default case that sets all motors to STOP --- But we should make it go forward 0.5m
        else:
            return cases[stopmode]