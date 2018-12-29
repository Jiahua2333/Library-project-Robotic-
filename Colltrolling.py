from IRobotProcessing import IRobotProcessing
from Tkinter import *
import math
import numpy as np
# import tkMessageBox
# import tkSimpleDialog

"""
This program uses keyboard to control the robot move
Attention: when the panel appears on the screen, it must click inside
           one time first(that is nothing), then, it can use the keyboard 
            control the robot.
"""
class KeyBoardController():
    robot = IRobotProcessing()
    file = open('Distance_Angle.txt', 'w')
    preRightEncoder = 0
    preLeftEncoder = 0
    segmentDistance = 0
    cornerRotatedAngle = 0
    '''
    def __init_(self):
        self.robot = IRobotProcessing()
        self.file = open('Distance_Angle.txt', 'w')
        self.preRightEncoder = 0
        self.preLeftEncoder = 0
        self.segmentDistance = 0
        self.cornerRotatedAngle = 0
    '''

    def __init__(self):
        self.root = Tk()
        self.frame = Frame(self.root, width=100, height=100)
        self.frame.bind("<Key>", self.key)
        self.frame.bind("<KeyRelease>", self.keyRelease)
        self.frame.bind("<Button-1>", self.callback)
        self.frame.pack()
        self.root.mainloop()

    def callback(self, event):
        self.frame.focus_set()
        print "clicked at", event.x, event.y

    def key(self, event):
        keyboard = event.char

        if keyboard is 'w':
            self.moveFoward()
        elif keyboard is 's':
            self.moveBackward()
        elif keyboard is 'a':
            self.counterclockwiseRotation()
        elif keyboard is 'd':
            self.clockwiseRotation()
        elif keyboard is 'h':
            self.save()
        elif keyboard is 'e':
            self.close()
        elif keyboard == 'c':
            self.clear()

    def keyRelease(self, event):
        self.robot.move(0, 0)

    def moveFoward(self):
        self.robot.move(100, 100)
        self.record()

    def moveBackward(self):
        self.robot.move(-100, -100)
        self.record()

    def counterclockwiseRotation(self):
        self.robot.move(20, -20)
        self.record()

    def clockwiseRotation(self):
        self.robot.move(-20, 20)
        self.record()

    def close(self):
        self.root.destroy()
        self.file.close()
        self.robot.reset()
        self.robot.close()

    def clear(self):
        self.preRightEncoder = 0
        self.preLeftEncoder = 0

    def save(self):
        if self.cornerRotatedAngle > 30 or self.cornerRotatedAngle < -30:
            self.file.write(str(self.cornerRotatedAngle) + '\n')
        else:
            self.file.write(str(0) + '\n')

        if self.segmentDistance > 300 or self.segmentDistance < -300:
            self.file.write(str(self.segmentDistance) + '\n')
        else:
            self.file.write(str(0) + '\n')
        self.cornerRotatedAngle = 0
        self.segmentDistance = 0

    def record(self):
        distance, angle, right_distance, left_distance, right_encoder, left_encoder = self.robot.getDistanceAngle(self.preRightEncoder, self.preLeftEncoder)
        self.preRightEncoder = right_encoder
        self.preLeftEncoder = left_encoder
        # if angle > 20 or angle < -20:
        self.cornerRotatedAngle = angle + self.cornerRotatedAngle

        self.segmentDistance = distance + self.segmentDistance
        # print "angle " + str(self.cornerRotatedAngle)
        print "Distance :" + str(self.segmentDistance)


if __name__ == "__main__":
    app = KeyBoardController()



"""
def key(event):
    print 'Pressed', repr(event.char)
    if event.char is 'g':
        run(vr, vl)
    elif event.char is 's':
        saveDistanceAngle(vr, vl)

    keyboard = event.char


def keyRelease(event):
    print 'Released', repr(event.char)
    stop()
    # print event.type
    # print type(event.char)
    # print event.char

def callback(event):
    frame.focus_set()
    print "clicked at", event.x, event.y

def run(right, left):
    bot.setWheelSpeed(right, left)

def stop():
    bot.setTurnSpeed(0)

def reset():
    bot.reset()

def saveDistanceAngle(vr, vl):
    right_encoder, left_encoder = bot.getEncoderCounts()
    if vr < 0:
        right_encoder = 65536 - right_encoder
        right_distance = -right_encoder / 508.8 * np.pi * 72
        left_distance = left_encoder / 508.8 * np.pi * 72
    else:
        left_distance = left_encoder / 508.8 * np.pi * 72
        right_distance = right_encoder / 508.8 * np.pi * 72
    if vl < 0:
        left_encoder = 65536 - left_encoder
        right_distance = right_encoder / 508.8 * np.pi * 72
        left_distance = -left_encoder / 508.8 * np.pi * 72

    angle = math.degrees((right_distance - left_distance) / 235.0)
    distance = (right_distance+left_distance)/2
    print distance
    print angle
    f.write(str(distance) + '\n')
    f.write(str(angle) + '\n')

    '''
    if angle > 360:
        angle = 65536 - angle
    if angle > 5:
        f.write(str(angle) + '\n')
    else:
        f.write('0\n')
    '''

vr = 100
vl = 100
f = open('Distance_Angle.txt', 'w')
bot = Robot()
root = Tk()
frame = Frame(root, width=100, height=100)
frame.bind("<Key>", key)
frame.bind("<KeyRelease>", keyRelease)
frame.bind("<Button-1>", callback)
frame.pack()

root.mainloop()
file.closed
"""