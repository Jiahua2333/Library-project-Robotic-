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

    # setup the GUI frame
    def __init__(self):
        self.root = Tk()
        self.frame = Frame(self.root, width=100, height=100)
        self.frame.bind("<Key>", self.key)
        self.frame.bind("<KeyRelease>", self.keyRelease)
        self.frame.bind("<Button-1>", self.callback)
        self.frame.pack()
        self.root.mainloop()
    
    # setup the callback reactivity for keyboard.
    def callback(self, event):
        self.frame.focus_set()
        print "clicked at", event.x, event.y
           
    # setup the motions for the certaion keys.
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
        self.robot.move(0, 0) # when the key is released, the wheels speed set to 0.

    def moveFoward(self):
        self.robot.move(100, 100) # Set the wheels speed to 100 mm/s. The parameter: (right wheel, left wheel)
        self.record()             # Record the distance and the angle of the robot moved

    def moveBackward(self):
        self.robot.move(-100, -100) # Set the wheels speed to -100 mm/s
        self.record()               # Record the distance and the angle of the robot moved

    def counterclockwiseRotation(self):
        self.robot.move(20, -20)    # Turn left
        self.record()

    def clockwiseRotation(self):
        self.robot.move(-20, 20)    # Turn right
        self.record()

    # Close the robot
    def close(self):
        self.root.destroy()
        self.file.close()
        self.robot.reset()
        self.robot.close()

    # Clear encoder to 0
    def clear(self):
        self.preRightEncoder = 0
        self.preLeftEncoder = 0
    
    # Save the total distance and angle to file
    def save(self):
        # Check if it went straight.
        if self.cornerRotatedAngle > 30 or self.cornerRotatedAngle < -30:
            self.file.write(str(self.cornerRotatedAngle) + '\n')
        else:
            self.file.write(str(0) + '\n')

        # Check if it turned
        if self.segmentDistance > 300 or self.segmentDistance < -300:
            self.file.write(str(self.segmentDistance) + '\n')
        else:
            self.file.write(str(0) + '\n')
        self.cornerRotatedAngle = 0
        self.segmentDistance = 0

    def record(self):
        # Get the encoder and use the difference between current Encoders and preEncoders to calculate the distance and angle
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

