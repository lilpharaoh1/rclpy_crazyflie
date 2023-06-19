"""
Copyright (c) 2018, Joseph Sullivan
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the <project name> project.
"""

VELOCITY = 0.2
RATE = 360. / 5.
ANGLE_DEGREES = 360.

class Back:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class CircleLeft:
    def __init__(self, radius_m, velocity=VELOCITY, angle_degrees=ANGLE_DEGREES):
        self.radius_m = radius_m
        self.velocity = velocity
        self.angle_degrees = angle_degrees

class CircleRight:
    def __init__(self, radius_m, velocity=VELOCITY, angle_degrees=ANGLE_DEGREES):
        self.radius_m = radius_m
        self.velocity = velocity
        self.angle_degrees = angle_degrees

class Down:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class Forward:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class Land:
    def __init__(self, velocity=VELOCITY):
        self.velocity = velocity

class Left:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class MoveDistance:
    def __init__(self, distance_x_m, distance_y_m, distance_z_m, velocity=VELOCITY):
        self.x = distance_x_m
        self.y = distance_y_m
        self.z = distance_z_m
        self.velocity = velocity
class Right:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class TakeOff:
    def __init__(self, height=None, velocity=VELOCITY):
        self.height = height
        self.velocity = velocity

class TurnLeft:
    def __init__(self, angle_degrees, rate=RATE):
        self.angle_degrees = angle_degrees
        self.rate = rate

class TurnRight:
    def __init__(self, angle_degrees, rate=RATE):
        self.angle_degrees = angle_degrees
        self.rate = rate

class Up:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity