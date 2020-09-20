from PyQt5 import QtGui, QtCore, QtWidgets
import warnings
import math

# Interface for modified stepper controller from: https://github.com/totesalaz/MKM/blob/master/Motorized_Kinematic_Mount.ino
# The command 'SENS:#:ST?' must be added to read sensors connected to Arduino.
class stepper_controller():
    def __init__(self, port, n_motors):
        self.port = port
        self.n_motors = n_motors
    
    def get_status(self, motor):
        self.port.write('STPM:{:d}:ST?'.format(motor).encode('utf-8'))
        self.port.flush()
        reply = self.port.read_until()
        return [int(i) for i in reply.split(':')]

    def reset_pos(self,  motor):
        self.port.write('STPM:{:d}:RST'.format(motor).encode('utf-8'))

    def set_vel(self,  motor, vel):
        self.port.write('STPM:{:d}:VEL:{:d}'.format(motor, vel).encode('utf-8'))
        self.port.flush()

    def move_rel(self, motor, steps):
        self.port.write('STPM:{:d}:REL:{:d}'.format(motor, steps).encode('utf-8'))
        self.port.flush()

    def move_abs(self, motor, steps):
        self.port.write('STPM:{:d}:ABS:{:d}'.format(motor, steps).encode('utf-8'))
        self.port.flush()

    def get_sensor(self, sensor):
        self.port.write('SENS:{:d}:ST?'.format(sensor).encode('utf-8'))
        self.port.flush()
        reply = self.port.read_until()
        return float(reply)

# Emulates a microcontroller running MKM software to enable testing without hardware. Keeps track of axis positions.
class FakeSerial():
    def __init__(self, name):
        self.name = name
        self.last_cmd = ""
        self.pos = {}

    def write(self, cmd, *args):
        self.last_cmd = cmd.decode()
        split = self.last_cmd.split(':')
        if len(split) == 4 and split[0] == 'STPM':
            key = int(split[1])
            val = int(split[3])
            if split[2] == 'ABS':
                self.pos[key] = val
            elif split[2] == 'REL':
                if key in self.pos:
                    self.pos[key] += val
                else:
                    self.pos[key] = val

        return len(cmd)

    def read_until(self):
        split = self.last_cmd.split(':')
        if len(split) == 3 and split[0] == 'STPM' and split[2] == 'ST?':
            if split[1] in self.pos:
                return '{}:1:0'.format(self.pos[split[1]])
            else:
                return '0:1:0'
        elif len(split) == 3 and split[0] == 'SENS' and split[2] == 'ST?':
            return str(math.exp(-sum([0.0001*i**2 for i in self.pos.values()])))
        else:
            warn_str = "Reading from serial after invalid command {}!".format(self.last_cmd)
            warnings.warn(warn_str)
            return ""

    def close(self):
        pass

    def flush(self):
        pass
