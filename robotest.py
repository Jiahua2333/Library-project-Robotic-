#!/usr/bin/env python3

'''
robotest.py - Test the features of BreezyCreate2

This code is part of BreezyCreate2

The MIT License

Copyright (c) 2016 Simon D. Levy

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''

from breezycreate2 import Robot
import time
import numpy as np
import math

# Create a Create2. This will automatically try to connect to your robot over serial
bot = Robot()

# Play a note to let us know you're alive!
# +bot.playNote('A4', 100)

# Tell the Create2 to turn right slowly
pre_right_encoder, pre_left_encoder = bot.getEncoderCounts()
print pre_right_encoder, pre_left_encoder
# bot.setTurnSpeed(-20)
vl = 30
vr = 30
bot.setWheelSpeed(vr, vl)
time.sleep(5)


# bot.setTurnSpeed(20)
# time.sleep(2)

# bot.setForwardSpeed(200)
# time.sleep(5)

# distance = bot.getDistance()
# distance, angle = bot.getDisAng()
# print 'Angles = ' + str(angle)
# print angle
# print distance

right_encoder, left_encoder = bot.getEncoderCounts()
difference_right = right_encoder - pre_right_encoder
difference_left = left_encoder - pre_left_encoder

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

print right_encoder
print left_encoder

angle = math.degrees((right_distance-left_distance)/235.0)

print right_distance
print left_distance
print angle

# bot.setWheelSpeed(20, 20)

# time.sleep(5)

# bot.setForwardSpeed(-200)

# time.sleep(5)

# Stop
bot.setTurnSpeed(0)

'''
start_time = time.time()
while (time.time() - start_time) < 30:
    print('Bumpers: ' + str(bot.getBumpers()) + '    Wall: ' + str(bot.getWallSensor()))
'''


# print 'distance = ' + str(distance)


# Close the connection
# bot.reset()

bot.close()
