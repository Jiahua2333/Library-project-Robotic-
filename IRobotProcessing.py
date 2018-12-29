from breezycreate2 import Robot
import math
import numpy as np

"""
This program simplifies some robot's functions from
the "_init_.py" file and adds some news functions for 
the library project.
"""

class IRobotProcessing():

    def __init__(self):
        self.__vr = 0
        self.__vl = 0
        self.bot = Robot()

    def setVR(self, vr):
        self.__vr = vr

    def setVL(self, vl):
        self.__vl = vl

    def getVR(self):
        return self.__vr

    def getVL(self):
        return self.__vl

    def run(self):
        self.bot.setWheelSpeed(self.__vr, self.__vl)

    def move(self, vr, vl):
        self.bot.setWheelSpeed(vr, vl)

    def stop(self):
        self.bot.setWheelSpeed(0, 0)

    def close(self):
        self.bot.close()

    def reset(self):
        self.bot.reset()

    def getDistanceAngle(self, pre_right_encoder, pre_left_encoder):
        right_encoder, left_encoder = self.getEncoder()
        difference_right = right_encoder - pre_right_encoder
        difference_left = left_encoder - pre_left_encoder
        # right_distance = difference_right / 508.8 * np.pi * 72
        # left_distance = difference_left / 508.8 * np.pi * 72

        if difference_right > 60000:
            right_distance = ((right_encoder - 65536) - pre_right_encoder) / 508.8 * np.pi * 72
        elif difference_right < -60000:
            right_distance = ((65536 - pre_right_encoder) + right_encoder) / 508.8 * np.pi * 72
        else:
            right_distance = difference_right / 508.8 * np.pi * 72

        if difference_left > 60000:
            left_distance = ((left_encoder - 65536) - pre_left_encoder) / 508.8 * np.pi * 72
        elif difference_left < -60000:
            left_distance = ((65536 - pre_left_encoder) + left_encoder) / 508.8 * np.pi * 72
        else:
            left_distance = difference_left / 508.8 * np.pi * 72




        angle = math.degrees((right_distance - left_distance) / 235.0)
        distance = (right_distance+left_distance)/2

        return distance, angle, right_distance, left_distance, right_encoder, left_encoder

    def getEncoder(self):
        right_encoder, left_encoder = self.bot.getEncoderCounts()
        '''
        if vr < 0:
            right_encoder = 65536 - right_encoder
        if vl < 0:
            left_encoder = 65536 - left_encoder
        '''

        print "right encoder: " + str(right_encoder)
        print "left encoder: " + str(left_encoder)

        return right_encoder, left_encoder

    def lineFollowing(self, status, expectation_distance, pre_cx, cx, rotation_direction, small_distance, angle):
        if status == 1:
            if small_distance < expectation_distance:
                self.setVR(15)
                self.setVL(15)
                self.run()
            else:
                self.setVR(0)
                self.setVL(0)
                self.run()
                status = 0

            '''
            if angle >= 85 or angle < -85:
                status = 0
                distance = 0
                angle = 0
            '''
        else:
            if cx == -1:
                print 'Out of track'
                '''
                if rotation_direction < 0 or pre_cx < 320:
                    self.setVR(15)
                    self.setVL(-15)
                    self.run()
                elif rotation_direction > 0 or pre_cx > 320:
                    self.setVR(-15)
                    self.setVL(15)
                    self.run()
                else:
                    self.setVR(15)
                    self.setVL(15)
                    self.run()
                '''
                if angle > 0:
                    self.setVR(15)
                    self.setVL(-15)
                    self.run()
                else:
                    self.setVR(-15)
                    self.setVL(15)
                    self.run()

            else:
                if cx >= 340 and cx < 440:
                    self.setVR(5)
                    self.setVL(20)
                    self.run()
                    print 'Turning right'
                    # time.sleep(3)
                    # bot.setTurnSpeed(0)
                    # time.sleep(2)
                elif cx >= 440:
                    self.setVR(5)
                    self.setVL(25)
                    self.run()
                    print "Turning right"

                elif cx < 340 and cx > 300:
                    if rotation_direction > 0:
                        self.setVR(20)
                        self.setVL(5)
                        self.run()
                    elif rotation_direction < 0:
                        self.setVR(5)
                        self.setVL(25)
                        self.run()
                    else:
                        self.setVR(40)
                        self.setVL(40)
                        self.run()
                    print 'Going forward'
                    # time.sleep(1)
                    # bot.setTurnSpeed(0)
                    # time.sleep(2)
                elif cx <= 300 and cx > 200:
                    self.setVR(20)
                    self.setVL(5)
                    self.run()
                    print 'Turning left'
                    # time.sleep(3)
                    # bot.setTurnSpeed(0)
                    # time.sleep(2)
                elif cx <= 200:
                    self.setVR(25)
                    self.setVL(5)
                    self.run()
                    print "Turning left"
        if cx != -1:
            return cx, status
        else:
            return pre_cx, status