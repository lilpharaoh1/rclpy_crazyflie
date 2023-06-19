VELOCITY = 0.2
RATE = 360. / 5.

class StartBack:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class SetVelSetpoint:
    def __init__(self, vx, vy, vz, rate_yaw):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.rate_yaw = rate_yaw

class StartBack:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity
class StartCircleLeft:
    def __init__(self, radius_m, velocity = VELOCITY):
        self.radius_m = radius_m
        self.velocity = velocity

class StartCircleRight:
    def __init__(self, radius_m, velocity = VELOCITY):
        self.radius_m = radius_m
        self.velocity = velocity

class StartDown:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class StartForward:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class StartLeft:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class StartLinearMotion:
    def __init__(self, vx, vy, vz):
        self.vx = vx
        self.vy = vy
        self.vz = vz

class StartRight:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class StartTurnLeft:
    def __init__(self, rate = RATE):
        self.rate = rate

class StartTurnRight:
    def __init__(self, rate = RATE):
        self.rate = rate

class StartUp:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class Stop:
    def __init__(self):
        pass