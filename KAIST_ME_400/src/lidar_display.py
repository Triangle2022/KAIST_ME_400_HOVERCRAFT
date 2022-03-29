#!/usr/bin/env python3
from __future__ import print_function
import roslib
import sys
import rospy
import numpy as np
import math
import time
from adafruit_servokit import ServoKit


kit = ServoKit(channels=16)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

## image size setting
x_size = 1080
y_size = 720 

### the origin of the angle is center of the robot front
start_x_angle  = 0
end_x_angle = 60

start_x_angle = 0
end_y_angle = 60

#################
standard_x = x_size/2
standard_y = y_size/2

## color configure
white = (255,255,255)
red = (0,0,255)
green = (0,255,0)
blue = (255,0,0)
starnge = (102,58,255)

## other parameters

## list of the x,y angle
x_y_coordinate = {}
start_angle_l = 0
end_angle_l = 270

start_angle_r = 540
end_angle_r = 660

angle_IMU = 0
gyro_IMU = 0

#constants used in PID control for steering
steerKp = 8
steerKd = 0.4

steerPreviousError = 0
steerPreviousTime = 0

lidarPreviousError = 0 #distance difference error of last time callback updated error
lidarPreviousTime = time.time() #last time that callback updated time
lidarKp = 0.8
lidarKd = 0

#desired angle
targetAngle = 0
previousTargetAngle = 0

previousTime = 0
prev_fr_dist = 0

#speed of motor on a scale of 0 to 180
rear_motor = 0
servo_angle = 90

#Dictionaries used to get data of left and right walls
leftData = {}
rightData = {}


initial_lidar = False

#global variables used to tune the general angle the hovercraft is heading
headingAngle = 0
headingAngle_ref = 0
headingAngle_count = 100
headingAngle_count_max = 200

#speed of front and rear motors on a scale from 0 to 180
front_test = 100
rear_test = 40

#scales a value between fromMin to fromMax to a value between toMin and toMax
def newMap(value,fromMin,fromMax,toMin,toMax):
    return (value-fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin

#pid controller utilizing IMU data
def steerPidControl(setpoint, currentValue,current_Value_diff):
    global steerPreviousTime
    timeNow = time.time()
    error = setpoint - currentValue
    control = steerKp * error  -steerKd * current_Value_diff
    if(control < -60): control = -60
    if(control > 60): control = 60
    steerPreviousTime = timeNow
    steerPreviousError = error
    control = int(control)
    return control

class core_processing:

    def __init__(self):
        global kit



        time.sleep(10.0)



        ## initalize the esc of the motor
        kit.servo[4].angle = 0
        kit.servo[11].angle = 0
        kit.servo[15].angle = 90


        time.sleep(1.0)
        self.image_sub = rospy.Subscriber("scan",LaserScan,self.callback) #subscriber that receives topic 'scan'
        self.imu_sub = rospy.Subscriber("yaw_data",Float32MultiArray,self.callback2)#subscriber that recieves 'yaw data'
        self.loop_sub = rospy.Subscriber("loop_gen",Int32,self.callback3)#callback3 processes data to decide steering angle

        #publish speed and angle of motors and servos
        self.front = rospy.Publisher('front_motor',Int32,queue_size=10)
        self.rear = rospy.Publisher('rear_motor',Int32,queue_size=10)
        self.servo = rospy.Publisher('servo_motor',Int32,queue_size=10)

        

    #stores data of individual points in dictionary 'x_y_coordinate'
    def cal_draw_location(self,angle,range):

        global x_y_coordinate
        global start_x
        global start_y
        global end_x
        global end_y

        theta = angle*(math.pi/573) 
        x = -(math.sin(theta) *range)
        y = math.cos(theta) *range
        try:
            x = int(x*100)
        except :
            x = 0
        
        try:
            y = int(y*100)
        except:
            y = 0
        
        x_y_coordinate[str(angle)] = [x,y]

        if(angle == 40):
            pass

        return x,y

    #functions 'draw_lidar_point' and 'draw_line' are used for visualization and debugging
    def draw_lidar_point(self,img,x,y,point_size,color):
        ## draw lidar point
        cv2.circle(img,(x+standard_x,-y+standard_y),point_size,color,-1)

    def draw_line(self,img,point1,point2):

        ## transform the point to the normal X_Y coordinate
        a1 = point1[0]+standard_x
        b1 = -point1[1]+standard_y
        a2 = point2[0]+standard_x
        b2 = -point2[1]+standard_y

        ## draw the line
        cv2.line(img,(a1,b1),(a2,b2),red,2)

    
    def cal_dist(self,x1, y1, x2, y2, a, b):
        # calculate the disstance between point and line
        # (x1,y1) and (x2,y2) make the line , (a,b) is the point

        area = abs((x1-a) * (y2-b) - (y1-b) * (x2 - a))
        AB = ((x1-x2)**2 + (y1-y2)**2) **0.5
        try:
            distance = area/AB
        except:
            distance = 0
        return distance

    def make_point_and_line(self,data,start_angle,end_angle,lr,angle_ref,current_angle): #Draw the left or right line


        global x_y_coordinate
        global start_x
        global start_y
        global end_x
        global end_y

        ##initialize all data
        x_y_coordinate = {}
        start_x = 0 
        start_y = 0
        end_x = 0
        end_y = 0
        max_dist = 0
        max_index = 0
        max_thres_dist = 0
        fr_dist = 0
        cornerNum = 0

        min_group_distance = 40 #min_group distance will determine if there is a discontinuity in the lines

        #recieves data from 'scan' and stores in dictionary 'x_y_coordinate'
        for i in range(len(data.ranges)):
            ranges = data.ranges[i]
            self.cal_draw_location(i,ranges)
        
        
        
        # points used to determine the distance from the hovercraft's front to the wall
        fr_x1 = x_y_coordinate[str(4)][0]
        fr_y1 = x_y_coordinate[str(4)][1]
        fr_x2 = x_y_coordinate[str(1146-4)][0]
        fr_y2 = x_y_coordinate[str(1146-4)][1]

        fr_line_center_x = (fr_x1 + fr_x2)/2
        fr_line_center_y = (fr_y1 + fr_y2)/2
        fr_dist = math.sqrt(fr_line_center_x**2 + fr_line_center_y**2) #calculate distance to front line in the hovercraft's coordinate system
        


        #angle of hovercraft calculated from the track's coordinate system
        turn_angle = (angle_ref - current_angle)%360

        if(turn_angle<0):
            turn_angle = turn_angle + 360

        current_angle_to_lidar_angle = int(turn_angle*1146/360) + 4

        if(current_angle_to_lidar_angle>1146):
            current_angle_to_lidar_angle = current_angle_to_lidar_angle-1146

        turn_x1 = x_y_coordinate[str(current_angle_to_lidar_angle)][0]
        turn_y1 = x_y_coordinate[str(current_angle_to_lidar_angle)][1]
        turn_x2 = x_y_coordinate[str(1146-(current_angle_to_lidar_angle))][0]
        turn_y2 = x_y_coordinate[str(1146-(current_angle_to_lidar_angle))][1]

        turn_line_center_x = (turn_x1 + turn_x2)/2
        turn_line_center_y = (turn_y1 + turn_y2)/2
        turn_dist = math.sqrt(turn_line_center_x**2 + turn_line_center_y**2) #calculate distance to front line in the track's coordinate system


        
        #here we put code to find out discontinuites and abandon discontinued lines
        #we calculate the distance of every point from the point before and if it exceeds a certain threshold we abandon every point after it
        
        if(end_angle == 270):
            angle = end_angle
            dist = 0
            prev_x = x_y_coordinate[str(end_angle)][0]
            prev_y = x_y_coordinate[str(end_angle)][1]
            while(dist < min_group_distance and angle >= 4):
                a = x_y_coordinate[str(angle)][0]
                b = x_y_coordinate[str(angle)][1] 
                dist = math.sqrt((a-prev_x)**2 + (b-prev_y)**2)
                if(dist >= min_group_distance):
                    start_angle = angle + 1
                angle -= 1
                prev_x = a
                prev_y = b
        
        if(start_angle == 1146-270):
            angle = start_angle
            dist = 0
            prev_x = x_y_coordinate[str(start_angle)][0]
            prev_y = x_y_coordinate[str(start_angle)][1]
            while(dist < min_group_distance and angle <= 1146-4):
                a = x_y_coordinate[str(angle)][0]
                b = x_y_coordinate[str(angle)][1] 
                dist = math.sqrt((a-prev_x)**2 + (b-prev_y)**2)
                if(dist >= min_group_distance):
                    end_angle = angle - 1
                angle += 1
                prev_x = a
                prev_y = b
        
        #xy coordinate of start point
        x1 = x_y_coordinate[str(start_angle+1)][0]
        y1 = x_y_coordinate[str(start_angle+1)][1]

        #xy coordinate of end point
        x2 = x_y_coordinate[str(end_angle-1)][0]
        y2 = x_y_coordinate[str(end_angle-1)][1]

        for j in range(start_angle,end_angle): #Get a point to draw a corner
            a = x_y_coordinate[str(j)][0]
            b = x_y_coordinate[str(j)][1] 
            dist = self.cal_dist(x1,y1,x2,y2,a,b)
            thres_dist = (a*a+b*b)
            if(dist > max_dist and a != 0 and thres_dist > max_thres_dist and dist < 120):
                max_thres_dist = thres_dist
                max_dist = dist
                max_index = j

        


        #define the coordinate of the corner point
        x3 = x_y_coordinate[str(max_index)][0]
        y3 = x_y_coordinate[str(max_index)][1]
        
        #if the max distance of the corner point relative to the primary line is smaller than 5, it is not considered as a corner
        if(max_dist>5):
            cornerNum = 1
            if(lr == 'right'):
                line_dist = self.cal_dist(x1,y1,x_y_coordinate[str(max_index)][0],x_y_coordinate[str(max_index)][1],0,0)
            if(lr == 'left'):
                line_dist = self.cal_dist(x_y_coordinate[str(max_index)][0],x_y_coordinate[str(max_index)][1],x2,y2,0,0)
        
        else:
            cornerNum = 0
            line_dist = self.cal_dist(x1,y1,x2,y2,0,0)
        
        #return a dictionary with info about number of corners, distance from the front wall in the hovercraft's coordinate,
        #distance from the front wall in the track's coordinate, and the distance from the side wall
        #also return 3 points(start, corner, end) if there is a corner, and return two points(start, end) if there is no corner
        return_dict = {}
        return_dict['cornerNum'] = cornerNum
        return_dict['fr_dist'] = fr_dist
        return_dict['turn_dist'] = turn_dist
        return_dict['line_dist'] = line_dist
        if(lr == 'left'):
            try:
                return_dict['points_1'] = (x1,y1)
                return_dict['points_2'] = (x3,y3)
                return_dict['points_3'] = (x2,y2)
            except:
                return_dict['points_1'] = (x1,y1)
                return_dict['points_2'] = (x2,y2)

        elif(lr == 'right'):
            try:
                return_dict['points_1'] = (x2,y2)
                return_dict['points_2'] = (x3,y3)
                return_dict['points_3'] = (x1,y1)
            except:
                return_dict['points_1'] = (x2,y2)
                return_dict['points_2'] = (x1,y1)
        return return_dict 




    def callback(self,data):

        global headingAngle_ref
        global angle_IMU
        global initial_lidar


        # the 0-360 scale is 0-1146
        start = time.time()
        leftData = self.make_point_and_line(data,4,270,'left',headingAngle_ref,angle_IMU) #This value must be clock wise -> get data of the left line
        rightData = self.make_point_and_line(data,1146-270,1146-4,'right',headingAngle_ref,angle_IMU)  #This value must be clock wise -> get data of the right line
        
        
        if(initial_lidar==False):
            initial_lidar = True
        
        
        
    #get yaw data from IMU sensor
    def callback2(self,data):
        global angle_IMU
        global gyro_IMU
        angle_IMU = data.data[0]
        gyro_IMU = data.data[1]
        pass
    
    #process data to determine actual steering angle
    def callback3(self,data):
        #To do : implement a callback3 function that can control the motor based on the lidar and imu Data

        # Variable That you can use
        # angle_IMU 
        # gyro_IMU 
        #

        # rear_motor -> 41 ~ 180
        # front_motor -> 41 ~ 180
        # servo_angle -> 0~180 , 90 is midpoint 

        rear_motor = 42
        front_motor = 42
        servo_angle  = 90

        
        ####################################
        ######## Your code here ############
        ####################################


        kit.servo[4].angle = rear_motor 
        kit.servo[11].angle = front_motor
        kit.servo[15].angle = servo_angle 

        
        pass


def main(args):
    rospy.init_node('core', anonymous=True)
    cp = core_processing()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
   
if __name__ == '__main__':
    main(sys.argv)


#### system brief

# ros_sub(in the init function ) -> callback(left,right,display)
