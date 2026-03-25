#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from clover_yolo.msg import Detection, DetectionArray
import cv2
import numpy as np
from math import *
import math
import time
from collections import deque

rospy.init_node('clover_c')

current_state = State()
bridge = CvBridge()

last_time = 0
takeoffed = False
rate = rospy.Rate(10)

detection_persons = []
detection_obstacles = []

drone_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

prev_frame = None

# Трекинг
target = None
prev_positions = deque(maxlen=10)
tracking = False

# PID переменные
error_x_prev = 0.0
error_y_prev = 0.0
error_z_prev = 0.0
integral_x = 0.0
integral_y = 0.0
integral_z = 0.0

# ========== НАСТРОЙКИ PID ==========
#KP_X, KI_X, KD_X = -0.004, 0.002, 0.001
#KP_Y, KI_Y, KD_Y = -0.06, 0.001, 0.0002
#KP_Z, KI_Z, KD_Z = 0.4, 0.003, 0.0

KP_X, KI_X, KD_X = -0.0018, 0.0012, 0.0004
KP_Y, KI_Y, KD_Y = -0.06, 0.001, 0.0002
KP_Z, KI_Z, KD_Z = 0.17, 0.0015, 0.0

# Целевая высота
TARGET_Z_MIN = 0.4
TARGET_Z_MAX = 0.7
TARGET_Z = 0.55

prev_vx_smooth = 0.0
prev_vy_smooth = 0.0
prev_vz_smooth = 0.0
SMOOTH_FACTOR = 0.3

WIDTH_CAMERA, HEIGHT_CAMERA = 640, 480
CENTER_X_CAMERA, CENTER_Y_CAMERA = WIDTH_CAMERA/2, HEIGHT_CAMERA/2
TARGET_DISTANCE = 1.7

# Ограничения
MAX_VELOCITY = 0.5
MIN_VELOCITY = 0.05

DEADZONE = 5.0

# Дальномер
rangefinder_distance = 0
RANGEFINDER_MAX_DISTANCE = 4.0

# ROS объекты
vel_pub = None
pos_pub = None
arming_client = None
set_mode_client = None

latest_image = None

TIMEOUT_FIND_TARGET = 15

states_tracking = {'not founded':0, 'track':1, 'missed':2}
state= states_tracking['not founded']

def position_callback(msg):
    global drone_pos
    drone_pos['x'] = msg.pose.position.x
    drone_pos['y'] = msg.pose.position.y
    drone_pos['z'] = msg.pose.position.z
    q = msg.pose.orientation
    drone_pos['roll'] = 0
    drone_pos['pitch'] = 0    
    drone_pos['yaw'] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    
def state_cb(msg):
    global current_state
    current_state = msg

def image_callback_front(msg):
    global latest_image, prev_frame
    try:
        latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"CV Bridge error: {e}")

def image_callback_main(msg):
    pass

def DetectionsCallback(msg):
    global detection_persons, detection_obstacles, target, state, states_tracking, takeoffed

    detection_persons = [det for det in msg.detections if det.class_name == 'person']
    detection_obstacles = [det for det in msg.detections if det.class_name != 'person']

    if takeoffed:
        #rospy.loginfo('Search target')
        currentTarget = None
        start_time = time.time()
        currentTarget = select_target(detection_persons)

        if currentTarget == None and target == None and state == states_tracking['not founded']:
            rospy.loginfo(f'Target not founded')
            state = states_tracking['not founded']
        elif currentTarget == None and target != None:
            rospy.loginfo('Target was missed')
            state = states_tracking['missed']
        elif currentTarget != None and state == states_tracking['missed']:
            rospy.loginfo('Target was return')
            state = states_tracking['track']
        elif currentTarget != None and target == None:
            rospy.loginfo(f'Find Target! Target pos in display = min({currentTarget.x_min}, {currentTarget.y_min}); max({currentTarget.x_max}, {currentTarget.y_max})')
            rospy.loginfo(f'Start tracking target')
            state = states_tracking['track']

        target = currentTarget

    #rospy.sleep(2)
    
def RangefinderCallback(msg):
    global rangefinder_distance
    rangefinder_distance = msg.range

state_sub = rospy.Subscriber('mavros/state', State, state_cb)
rospy.Subscriber('/main_camera/image_raw', Image, image_callback_main, queue_size=1)
rospy.Subscriber('/vision/debug_image', Image, image_callback_front)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, position_callback)
rospy.Subscriber('/vision/detections', DetectionArray, DetectionsCallback)
rospy.Subscriber('/rangefinder/range', Range, RangefinderCallback)

local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

def init_ros():
    global vel_pub, pos_pub, arming_client, set_mode_client
    
    rospy.wait_for_service('/mavros/set_mode', timeout=30)
    rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
    
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    rospy.loginfo("ROS services initialized")

def select_target(detection_persons):
    global prev_positions
    
    if not detection_persons or len(detection_persons) == 0:
        return None
    
    if prev_positions and len(prev_positions) > 0:
        prev_x, prev_y = prev_positions[-1]
        
        for det in detection_persons:
            center_x = (det.x_min + det.x_max) / 2
            center_y = (det.y_min + det.y_max) / 2
            dist = math.sqrt((center_x - prev_x)**2 + (center_y - prev_y)**2)
            if dist < 100:
                return det
    
    return detection_persons[0] if detection_persons else None

def pid_controller(error_x, error_y, error_z, dt):
    global error_x_prev, error_y_prev, error_z_prev
    global integral_x, integral_y, integral_z
    
    integral_x += error_x * dt
    integral_x = np.clip(integral_x, -10, 10)

    #derivative_x = (error_x - error_x_prev) / dt if dt > 0 else 0
    derivative_x = (error_x - error_x_prev) / dt
    
    error_x_prev = error_x

    vx = KP_X * error_x + KI_X * integral_x + KD_X * derivative_x
    
    integral_y += error_y * dt
    integral_y = np.clip(integral_y, -10, 10)

    #derivative_y = (error_y - error_y_prev) / dt if dt > 0 else 0
    derivative_y = (error_y - error_y_prev) / dt

    error_y_prev = error_y

    vy = KP_Y * error_y + KI_Y * integral_y + KD_Y * derivative_y
    
    integral_z += error_z * dt
    integral_z = np.clip(integral_z, -10, 10)

    #derivative_z = (error_z - error_z_prev) / dt if dt > 0 else 0
    derivative_z = (error_z - error_z_prev) / dt

    error_z_prev = error_z
    
    vz = KP_Z * error_z + KI_Z * integral_z + KD_Z * derivative_z
    
    return vx, vy, vz

def send_velocity_command(vx, vy, vz):
    global vel_pub
    
    if vel_pub is None:
        return
    
    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = "local_origin"
    
    twist.twist.linear.x = vx
    twist.twist.linear.y = vy
    twist.twist.linear.z = vz
    twist.twist.angular.z = 0.0
    
    vel_pub.publish(twist)

    #rospy.loginfo(f'Send velocitity drone x:{vx}, y:{vy}, z:{vz}')

def yaw_to_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def publish_setpoint(x=None, y=None, z=None, yaw=None):
    if x is None: x = drone_pos['x']
    if y is None: y = drone_pos['y']
    if z is None: z = drone_pos['z']
    if yaw is None: yaw = drone_pos['yaw']
    
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    q = yaw_to_quaternion(yaw)
    pose.pose.orientation = q
    
    local_pos_pub.publish(pose)
    return pose

def Takeoff():
    global takeoffed
    #yaw_init = 1.57
    yaw_init = 0

    for i in range(100):
        publish_setpoint(0, 0, TARGET_Z, yaw_init)
        rospy.sleep(0.02)

    rospy.wait_for_service('/mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    for i in range(20):
        set_mode_client(custom_mode="OFFBOARD")
        if current_state.mode == "OFFBOARD":
            break
        rospy.sleep(0.1)

    rospy.wait_for_service('/mavros/cmd/arming')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    arm_resp = arming_client(True)
    if not arm_resp.success:
        rospy.logerr("Arming failed!")
        return

    rospy.loginfo("Armed! Taking off...")
    rate_takeoff = rospy.Rate(20)
    for _ in range(200):
        publish_setpoint(0, 0, TARGET_Z, yaw_init)
        rate_takeoff.sleep()

    takeoffed = True

def Land():
    rospy.loginfo("Landing")
    rate_land = rospy.Rate(10)
    for _ in range(70): 
        publish_setpoint(0, 0, 0.2, 0)
        rate_land.sleep()
    rospy.sleep(3)
    rospy.wait_for_service('/mavros/set_mode')
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    set_mode(custom_mode="AUTO.LAND")

def Connect():
    rospy.loginfo("Waiting for PX4 connection...")
    while not current_state.connected and not rospy.is_shutdown():
        rospy.sleep(0.1)
    rospy.loginfo("Connected!")

def DelayPosition(delays, pos_x, pos_y, pos_z, yaw, pitch=0, roll=0):
    global rate
    start_time = time.time()
    while time.time() - start_time < delays and not rospy.is_shutdown():
        publish_setpoint(pos_x, pos_y, pos_z, yaw)
        rate.sleep()

def Delay(delays):
    DelayPosition(delays, drone_pos['x'], drone_pos['y'], drone_pos['z'], drone_pos['yaw'])

# ========== MAIN ==========

try:
        Connect()
        init_ros()
        Takeoff()

        init_yaw = drone_pos['yaw']
        
        #DelayPosition(4, 2*0.9, 0, TARGET_Z, init_yaw)
        DelayPosition(4, 0, 2*0.9, TARGET_Z, init_yaw)
        
        prev_frame = latest_image.copy() if latest_image is not None else None
        frame_count = 0
        
        rospy.loginfo("Starting visual tracking...")

        y_missed_pos = 0
        
        while not rospy.is_shutdown():
            frame_count += 1
            current_time = time.time()
            dt = current_time - last_time if last_time > 0 else 0.1
            last_time = current_time
            
            if frame_count % 50 == 0:
                #rospy.logwarn("Safety pause")
                #end_velocity_command(0, 0, 0)
                #rospy.sleep(1)
                rate.sleep()

            error_x = 0
            error_y = 0
            error_z = TARGET_Z - drone_pos['z']

            if state != states_tracking['not founded']:
                error_x_display = 0
                error_y_display = 0

                if state == states_tracking['track']:
                    target_center_x = (target.x_min + target.x_max) / 2
                    target_center_y = (target.y_min + target.y_max) / 2

                    error_x_display, error_y_display = CENTER_X_CAMERA - target_center_x, CENTER_Y_CAMERA - target_center_y
                    if abs(error_x_display) < DEADZONE:
                        error_x_display = 0.0
                    if abs(error_y_display) < DEADZONE:
                        error_y_display = 0.0
                    
                    error_x = error_x_display

                    if target_center_x > target.x_min - DEADZONE and target_center_x < target.x_max + DEADZONE:
                        error_y = TARGET_DISTANCE - rangefinder_distance
                    else:
                        error_y = 0
                        error_x *= 1.8

                    rate.sleep()
                elif state == states_tracking['missed']:
                    error_x = error_x_prev / 1.3
                    error_y = 0

                    if rangefinder_distance < 0.7:
                        error_y = 0.5
                    
                    rate.sleep()

            if drone_pos['z'] < TARGET_Z_MIN or drone_pos['z'] > TARGET_Z_MAX:
                    error_z *= 5

            #vx, vy, vz = pid_controller(error_x, error_y, error_z, dt)

            vy, vx, vz = pid_controller(error_x, error_y, error_z, dt)
            vy = -vy

            send_velocity_command(vx, vy, vz)
            
except KeyboardInterrupt:
        rospy.logerr("Emergency stop!")
        send_velocity_command(0, 0, 0)
finally:
        Land()
        cv2.destroyAllWindows()
        rospy.loginfo("Done")