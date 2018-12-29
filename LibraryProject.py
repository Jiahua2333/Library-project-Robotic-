# from breezycreate2 import Robot
import numpy as np
import cv2
from IRobotProcessing import IRobotProcessing
import matplotlib.pyplot as plt
import math
import time
# import math

"""
This class is about some openCV image processing. 
"""
class OpenCVProcessing():

    def __init__(self):
        self.img = None

    def setIMG(self, image):
        self.img = image

    def getIMG(self):
        return self.img

    def toGraySclace(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    def blurProcess(self, gray):
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        return blur

    def getCanny(sel, blur):
        edges = cv2.Canny(blur, 50, 100, apertureSize=3)
        return edges

    def getContours(self, blur_image):
        # blur = self.blurProcess()
        ret, thresh = cv2.threshold(blur_image, 10, 255, cv2.THRESH_BINARY)
        # mask = cv2.erode(thresh, None, iterations=2)
        # mask = cv2.dilate(mask, None, iterations=2)
        contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

        return contours, thresh

    def drawContours(self, blur_image, original_image):
        contours, thresh = self.getContours(blur_image)

        if len(contours) > 0:

            c = max(contours, key=cv2.contourArea)

            if cv2.contourArea(c) > 4000:

                M = cv2.moments(c)

                if M['m00'] == 0:
                    print 'error'
                    return original_image, -1
                else:
                    cx = int(M['m10'] / M['m00'])

                    cy = int(M['m01'] / M['m00'])

                    cv2.line(original_image, (cx, 0), (cx, 720), (255, 0, 0), 1)

                    cv2.line(original_image, (0, cy), (1280, cy), (255, 0, 0), 1)

                    cv2.drawContours(original_image, contours, -1, (0, 255, 0), 1)

                    return original_image, cx
            else:
                return original_image, -1
        else:
            return original_image, -1

    def getHoughTransformLines(self, blur_image):
        edges = self.getCanny(blur_image)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 150)  # type: vector

        return lines, edges

    def drawHoughLines(self, blur_image, img):
        lines, edges = self.getHoughTransformLines(blur_image)
        isHorizontalLine = False
        isHoughLines = False
        if lines is None:
            print 'No hough lines'
            return 0, img, edges, isHorizontalLine, isHoughLines

        avg_theta_vertical = 0.0
        # avg_theta_horizontal = 0.0
        rotation_direction = 0
        countVertical = 0.0
        # countHorizontal = 0.0

        for rho, theta in lines[0]:

            if theta * 180 / np.pi <= 45 or theta * 180 / np.pi >= 145:
                if rho >= 0:
                    avg_theta_vertical = avg_theta_vertical + theta * 180 / np.pi
                    rotation_direction += 1
                else:
                    avg_theta_vertical = avg_theta_vertical - (180 - theta * 180 / np.pi)
                    rotation_direction = rotation_direction - 1

                isHoughLines = True
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * a)
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * a)

                countVertical += 1
                # total_vertical_rho = total_vertical_rho + rho

                cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                print 'V: ' + str(theta * 180 / np.pi)
            elif theta * 180 / np.pi >= 80 and theta * 180 / np.pi <= 100:
                # avg_theta_horizontal = avg_theta_horizontal + theta*180/np.pi

                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * a)
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * a)

                # countHorizontal += 1
                cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                isHorizontalLine = True
                isHoughLines = True
                print 'H: ' + str(theta * 180 / np.pi)
        if countVertical != 0:
            avg_theta_vertical = avg_theta_vertical / countVertical
        '''
        if countHorizontal != 0:
            avg_theta_horizontal =avg_theta_horizontal/countHorizontal
        '''

        return rotation_direction, img, edges, isHorizontalLine, isHoughLines

    def getBlueRegion(self):
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        res = cv2.bitwise_and(self.img, self.img, mask=mask)

        return mask, res

    def getCoordinates(self, distance, angle, direction, xs, ys):
        if direction is "up":
            x, y = self.calCoordinates(90, distance, angle, xs, ys)
        elif direction is "down":
            x, y = self.calCoordinates(-90, distance, angle, xs, ys)
        elif direction is "right":
            x, y = self.calCoordinates(0, distance, angle, xs, ys)
        elif direction is "left":
            x, y = self.calCoordinates(180, distance, angle, xs, ys)

        return x, y


    def calCoordinates(self, adjustNum, distance, angle, xs, ys):
        if len(xs) == 0:
            x = (oneFrameDistance * np.cos(math.radians(oneFrameAngle + adjustNum)))
            y = (oneFrameDistance * np.sin(math.radians(oneFrameAngle + adjustNum)))
        else:
            x = (oneFrameDistance * np.cos(math.radians(oneFrameAngle + adjustNum)) + xs[len(xs) - 1])
            y = (oneFrameDistance * np.sin(math.radians(oneFrameAngle + adjustNum)) + ys[len(ys) - 1])

        return x, y


"""
This is the main function of the library project.
"""
if __name__ == "__main__":

    """
    Run this to read the data from files, then
    let the robot move.
    """
    video_live = cv2.VideoCapture(0)
    video_live.set(3, 640)  # Frame width
    video_live.set(4, 480)  # Frame height
    video_live.set(5, 1)  # Frames rate

    openCVProcessing = OpenCVProcessing()
    robot = IRobotProcessing()

    file = open('Distance_Angle.txt', 'r')
    file2 = open('result.txt', 'w')
    file3 = open("Angle_set.txt", 'w')
    file4 = open("Distance_set.txt", 'w')
    list = file.readlines()
    for i in range(0, len(list)):
        list[i] = list[i].rstrip('\n')

    plt.xlim(0, 2000)
    plt.ylim(0, 2000)
    plt.xlabel('X')
    plt.ylabel('Y')
    xs = []
    ys = []

    index = 0
    pre_cx = 320
    expectation_distance = 240
    status = 0
    small_distance = 0
    pre_right_encoder, pre_left_encoder = robot.getEncoder()

    while index < len(list):
        angle = float(list[index])
        distance = float(list[index + 1])

        if angle == 0.0:
            movement = 0
            while movement < distance:
                ret, img = video_live.read()
                openCVProcessing.setIMG(img)
                _, frame = openCVProcessing.getBlueRegion()
                gray = openCVProcessing.toGraySclace(frame)
                blur = openCVProcessing.blurProcess(gray)
                frame, cx = openCVProcessing.drawContours(blur, img)
                rotation_direction, img, edges, isHorizontal, isHoughLines = openCVProcessing.drawHoughLines(blur, img)
                if isHorizontal and movement >= (0.95*(distance-expectation_distance)):
                    status = 1
                pre_cx, status = robot.lineFollowing(status, expectation_distance, pre_cx, cx, rotation_direction, small_distance, angle)
                oneFrameDistance, oneFrameAngle, oneFrameRight, oneFrameLeft, rightEncoder, leftEncoder = robot.getDistanceAngle(pre_right_encoder, pre_left_encoder)
                pre_left_encoder = leftEncoder
                pre_right_encoder = rightEncoder

                file3.write(str(round(oneFrameAngle, 3)) + '\n')
                file4.write(str(round(oneFrameDistance, 3)) + '\n')

                '''
                x, y = openCVProcessing.getCoordinates(oneFrameDistance, oneFrameAngle, direction, xs, ys)
                xs.append(x)
                '''

                if status == 1:
                    small_distance = small_distance + oneFrameDistance
                    movement = movement + oneFrameDistance
                else:
                    movement = movement + oneFrameDistance

                if small_distance != 0:
                    if movement >= 0.93*distance:
                        break

                print 'Moving ' + str(movement) + ' mm'
                cv2.putText(img, "Angle: " + str(round(oneFrameAngle, 3)), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255))
                cv2.putText(img, "Distance: " + str(round(movement, 3)) + " mm", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

                cv2.imshow('result', img)
                cv2.imshow('Edges', edges)
                plt.plot(xs, ys, 'b')
                plt.pause(0.01)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            file2.write(str(movement) + '\n')
            status = 0
            small_distance = 0

        elif angle != 0 and distance == 0:
            movement = 0
            while abs(movement) < abs(angle):
                ret, img = video_live.read()
                openCVProcessing.setIMG(img)
                _, frame = openCVProcessing.getBlueRegion()
                gray = openCVProcessing.toGraySclace(frame)
                blur = openCVProcessing.blurProcess(gray)
                frame, cx = openCVProcessing.drawContours(blur, img)
                rotation_direction, img, edges, isHorizontal, isHoughLines = openCVProcessing.drawHoughLines(blur, img)
                if angle > 0 and movement < 60:
                    robot.setVR(11)
                    robot.setVL(-11)
                    robot.run()
                elif angle < 0 and movement > -60:
                    robot.setVR(-11)
                    robot.setVL(11)
                    robot.run()
                else:
                    robot.lineFollowing(status, expectation_distance, pre_cx, cx, rotation_direction, small_distance, angle)
                    if isHoughLines:
                        break
                oneFrameDistance, oneFrameAngle, oneFrameRight, oneFrameLeft, rightEncoder, leftEncoder = robot.getDistanceAngle(pre_right_encoder, pre_left_encoder)
                pre_left_encoder = leftEncoder
                pre_right_encoder = rightEncoder

                file3.write(str(round(oneFrameAngle, 3)) + '\n')
                file4.write(str(round(oneFrameDistance, 3)) + '\n')

                '''
                if len(xs) == 0:
                    xs.append(oneFrameDistance * np.cos(math.radians(oneFrameAngle)))
                    ys.append(oneFrameDistance * np.sin(math.radians(oneFrameAngle)))
                else:
                    xs.append(oneFrameDistance * np.cos(math.radians(oneFrameAngle)) + xs[len(xs) - 1])
                    ys.append(oneFrameDistance * np.sin(math.radians(oneFrameAngle)) + ys[len(ys) - 1])
                '''

                movement = movement + oneFrameAngle
                print "Rotated" + str(movement) + " degree"
                cv2.putText(img, "Angle: " + str(round(movement, 3)) + " degree", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255))
                cv2.putText(img, "Distance: 0 mm", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255))

                '''
                if abs(movement) > abs(0.98*angle):
                    break
                '''

                plt.plot(xs, ys, 'b')
                plt.pause(0.01)

                cv2.imshow('result', img)
                cv2.imshow('Edges', edges)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            file2.write(str(movement) + '\n')


        index = index + 2

    robot.stop()
    robot.reset()
    robot.close()
    file.close()
    file2.close()
    file3.close()
    file4.close()
    cv2.destroyAllWindows()

    """
    Run this for line following.
    """

    '''
    video_live = cv2.VideoCapture(0)
    video_live.set(3, 640)  # Frame width
    video_live.set(4, 480)  # Frame height
    video_live.set(5, 1)  # Frames rate

    openCVProcessing = OpenCVProcessing()
    robot = IRobotProcessing()

    plt.xlim(0, 2000)
    plt.ylim(0, 2000)
    plt.xlabel('X')
    plt.ylabel('Y')
    xs = []
    ys = []

    pre_cx = 320
    pre_right_encoder = 0
    pre_left_encoder = 0
    expectation_distance = 240
    distance = 0
    angle = 0
    right_distance = 0
    left_distance = 0
    total_distance = 0
    status = 0
    movement = ""

    while True:
        ret, img = video_live.read()
        openCVProcessing.setIMG(img)
        _, frame = openCVProcessing.getBlueRegion()
        gray = openCVProcessing.toGraySclace(frame)
        blur = openCVProcessing.blurProcess(gray)
        frame, cx = openCVProcessing.drawContours(blur, img)
        rotation_direction, img, edges, isHorizontalLine, isHoughLines = openCVProcessing.drawHoughLines(blur, img)
        if isHorizontalLine:
            status = 1

        if status == 1:
            if distance < expectation_distance:
                movement = "Going straight: " + str(distance) + " mm"
                cv2.putText(img, "Moving certain distance: " + str(distance) + " mm", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255))
                robot.setVR(15)
                robot.setVL(15)
                robot.run()
            else:
                if pre_cx < 320:
                    movement = "Rotating Left"
                    robot.setVR(20)
                    robot.setVL(-20)
                    robot.run()
                elif pre_cx >= 320:
                    movement = "Rotating Right"
                    robot.setVR(-20)
                    robot.setVL(20)
                    robot.run()

            if angle >= 85 or angle < -85:
                status = 0
                distance = 0
                angle = 0
        else:
            if cx == -1:
                print 'Out of track'
                movement = "Out of track"
                if rotation_direction < 0 or pre_cx <= 320:
                    robot.setVR(15)
                    robot.setVL(-15)
                    robot.run()
                elif rotation_direction > 0 or pre_cx > 320:
                    robot.setVR(-15)
                    robot.setVL(15)
                    robot.run()
                else:
                    robot.setVR(0)
                    robot.setVL(0)
                    robot.run()
            else:
                if cx >= 340 and cx < 440:
                    robot.setVR(5)
                    robot.setVL(20)
                    robot.run()
                    print 'Turning right'
                    movement = "Turning right"
                    # time.sleep(3)
                    # bot.setTurnSpeed(0)
                    # time.sleep(2)
                elif cx >= 440:
                    robot.setVR(5)
                    robot.setVL(30)
                    robot.run()
                    print "Turning right"
                    movement = "Turning right"

                elif cx < 340 and cx > 300:
                    if rotation_direction > 0:
                        robot.setVR(20)
                        robot.setVL(5)
                        robot.run()
                    elif rotation_direction < 0:
                        robot.setVR(5)
                        robot.setVL(25)
                        robot.run()
                    else:
                        robot.setVR(40)
                        robot.setVL(40)
                        robot.run()
                    print 'Going forward'
                    movement = "Going forward"
                    # time.sleep(1)
                    # bot.setTurnSpeed(0)
                    # time.sleep(2)
                elif cx <= 300 and cx > 200:
                    robot.setVR(20)
                    robot.setVL(5)
                    robot.run()
                    print 'Turning left'
                    movement = "Turning left"
                    # time.sleep(3)
                    # bot.setTurnSpeed(0)
                    # time.sleep(2)
                elif cx <= 200:
                    robot.setVR(30)
                    robot.setVL(5)
                    robot.run()
                    print "Turning left"
                    movement = "Turning left"

        oneFrameDistance, oneFrameAngle, oneFrameRight, oneFrameLeft, right_encoder, left_encoder = robot.getDistanceAngle(pre_right_encoder, pre_left_encoder)


        pre_right_encoder = right_encoder
        pre_left_encoder = left_encoder
        xs.append(oneFrameDistance * np.cos(oneFrameAngle))
        ys.append(oneFrameDistance * np.sin(oneFrameAngle))

        if status == 1:
            distance = distance + oneFrameDistance
            angle = oneFrameAngle + angle
            total_distance = total_distance + oneFrameDistance
            right_distance = oneFrameRight + right_distance
            left_distance = oneFrameLeft + left_distance

        else:
            total_distance = total_distance + oneFrameDistance
            right_distance = oneFrameRight + right_distance
            left_distance = oneFrameLeft + left_distance

        print "distance: " + str(distance)
        print "angle: " + str(angle)

        if cx != -1:
            pre_cx = cx

        cv2.putText(img, "Movement " + str(movement), (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        cv2.putText(img, "Total Distance: " + str(round(total_distance, 3)) + " mm", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        cv2.putText(img, "Angle: " + str(round(angle, 3)) + " degree", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))


        # cv2.imshow('frame', frame)
        # cv2.imshow('thresh', thresh)
        cv2.imshow('result', img)
        cv2.imshow('Edges', edges)
        # out.write(gray)
        # cv2.imshow('gray', gray)
        # cv2.imshow('blur', blur)
        plt.plot(xs, ys, 'b')
        plt.pause(0.05)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print "Total distance: " + str(total_distance)
    print 'Left wheel: ' + str(left_distance)
    print "Right wheel: " + str(right_distance)

    robot.stop()
    robot.reset()
    robot.close()
    # out.release()
    video_live.release()
    cv2.destroyAllWindows()
    '''