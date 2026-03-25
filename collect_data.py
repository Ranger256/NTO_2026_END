#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Автоматический полёт для сбора данных (на базе рабочего кода)
Облетает поле по сетке, меняет высоту, сохраняет изображения
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from datetime import datetime
from math import *
import math

# ========== НАСТРОЙКИ ПОЛЁТА ==========
MIN_ALTITUDE = 0.5      # Минимальная высота (м)
MAX_ALTITUDE = 1.5       # Максимальная высота (м)
ALTITUDE_STEPS = 4       # Количество уровней высоты
GRID_SIZE_X = 4.5*0.9       # Размер поля по X (м)
GRID_SIZE_Y = 3*0.9       # Размер поля по Y (м)
GRID_SPACING = 0.6       # Расстояние между линиями (м)
GRID_SPACING_X = 0.5
HOVER_TIME = 1.5         # Время зависания в точке (с)
SAVE_INTERVAL = 1.0      # Сохранять кадры каждые N секунд
# =======================================

rospy.init_node('data_collection_flight')

current_state = State()
bridge = CvBridge()
drone_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
image_count = 0
last_save_time = 0

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
SAVE_DIR = f"Datasets/Dataset_{timestamp}"
os.makedirs(SAVE_DIR, exist_ok=True)

# ========== CALLBACKS ==========
def position_callback(msg):
    global drone_pos
    drone_pos['x'] = msg.pose.position.x
    drone_pos['y'] = msg.pose.position.y
    drone_pos['z'] = msg.pose.position.z

def state_cb(msg):
    global current_state
    current_state = msg

def image_callback(msg):
    global image_count, last_save_time
    
    current_time = time.time()
    if current_time - last_save_time < SAVE_INTERVAL:
        return
    
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        filename = f"frame_{image_count:05d}.jpg"

        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

        filepath = os.path.join(SAVE_DIR, filename)
        
        cv2.imwrite(filepath, cv_image)
        
        #height, width = cv_image.shape[:2]
        #rospy.loginfo(f"[{image_count:4d}] {filename} ({width}x{height}) Z={drone_pos['z']:.2f}m")
        
        image_count += 1
        last_save_time = current_time
        
    except Exception as e:
        rospy.logerr(f"Error saving image: {e}")

# ========== SUBSCRIBERS & PUBLISHERS ==========
state_sub = rospy.Subscriber('mavros/state', State, state_cb)
rospy.Subscriber('/image_raw', Image, image_callback, queue_size=1)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, position_callback)
local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

# ========== ФУНКЦИИ ==========

def yaw_to_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def publish_setpoint(x, y, z, yaw=0):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    
    # Конвертация yaw в quaternion
    q = yaw_to_quaternion(yaw)
    pose.pose.orientation = q
    
    local_pos_pub.publish(pose)
    return pose

def Connect():
    rospy.loginfo("Waiting for PX4 connection...")
    rate = rospy.Rate(10)
    while not current_state.connected and not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo("Connected!")

def Takeoff(altitude):

    initial_yaw = math.pi / 2

    for i in range(100):
        publish_setpoint(0, 0, altitude, initial_yaw)
        rospy.sleep(0.02)

    rospy.wait_for_service('/mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    while current_state.mode != "OFFBOARD":
        set_mode_client(custom_mode="OFFBOARD")
        rospy.sleep(0.1)

    rospy.wait_for_service('/mavros/cmd/arming')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    arm_resp = arming_client(True)
    if not arm_resp.success:
        rospy.logerr("Arming failed!")
        return

    rospy.loginfo("Armed! Taking off...")

    rate = rospy.Rate(20)
    for _ in range(200):  # ~10 sec
        publish_setpoint(0, 0, altitude, initial_yaw)
        rate.sleep()

def Land():
    rospy.loginfo("Landing...")
    rospy.wait_for_service('/mavros/set_mode')
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    set_mode(custom_mode="AUTO.LAND")
    rospy.loginfo("Landing initiated!")

def GenerateWaypoints():
    """Генерация точек с поворотом на 90° и случайными изменениями yaw"""
    waypoints = []
    
    # Уровни высоты
    altitudes = [
        MIN_ALTITUDE + (MAX_ALTITUDE - MIN_ALTITUDE) * i / (ALTITUDE_STEPS - 1)
        for i in range(ALTITUDE_STEPS)
    ]
    
    rospy.loginfo(f"\nAltitude levels: {altitudes}")
    
    # Позиции по Y и X
    y_positions = [i * GRID_SPACING for i in range(int(GRID_SIZE_Y / GRID_SPACING) + 1)]
    
    # Базовый yaw = 90 градусов (π/2 радиан) - дрон повернут влево
    BASE_YAW = math.pi / 2  # 90 градусов
    
    for level_idx, altitude in enumerate(altitudes):
        rospy.loginfo(f"\n🔹 Level {level_idx + 1}/{ALTITUDE_STEPS}: {altitude:.1f}m")
        
        for i, y in enumerate(y_positions):
            # Чередование направления (змейка)
            if i % 2 == 0:
                x_start = 0
                x_end = GRID_SIZE_X
            else:
                x_start = GRID_SIZE_X
                x_end = 0
            
            num_points = int(abs(x_end - x_start) / GRID_SPACING_X) + 1
            x_positions = [x_start + (x_end - x_start) * j / max(num_points - 1, 1) for j in range(num_points)]
            
            for idx, x in enumerate(x_positions):
                # Поворот на 90° + небольшие изменения для разнообразия
                yaw = BASE_YAW
                
                # Каждые 5 точек немного меняем угол для разнообразия данных
                if idx % 5 == 0:
                    # Добавляем вариацию: -30°, 0°, +30°
                    variation_idx = (idx // 5) % 3
                    if variation_idx == 0:
                        yaw = BASE_YAW - math.pi / 6  # 60° (поворот чуть правее)
                    elif variation_idx == 1:
                        yaw = BASE_YAW  # 90° (базовый)
                    else:
                        yaw = BASE_YAW + math.pi / 6  # 120° (поворот чуть левее)
                
                # На разворотах (концы линий) добавляем дополнительные углы
                if idx == 0 or idx == len(x_positions) - 1:
                    # На концах линий смотрим чуть вперед/назад
                    if i % 4 == 0:
                        yaw = BASE_YAW - math.pi / 12  # 75°
                    else:
                        yaw = BASE_YAW + math.pi / 12  # 105°
                
                waypoints.append((x, y, altitude, yaw))
    
    rospy.loginfo(f"\nGenerated {len(waypoints)} waypoints with varied orientations")
    rospy.loginfo(f"Field: X [0, {GRID_SIZE_X}m], Y [0, {GRID_SIZE_Y}m]")
    rospy.loginfo(f"Base yaw: {math.degrees(BASE_YAW):.0f}° (rotated 90° left)")
    return waypoints

def FlyMission(waypoints):
    """Выполнение миссии с учетом yaw"""
    rate = rospy.Rate(20)
    
    for i, (x, y, z, yaw) in enumerate(waypoints):
        if rospy.is_shutdown():
            rospy.loginfo("\nMission interrupted!")
            return False
        
        # Конвертация yaw в градусы для логов
        yaw_deg = math.degrees(yaw)
        rospy.loginfo(f"WP {i+1}/{len(waypoints)}: ({x:.1f}, {y:.1f}, {z:.1f}m) yaw={yaw_deg:.0f}°")
        
        # Летим в точку
        start_time = time.time()
        while time.time() - start_time < 60:
            if rospy.is_shutdown():
                return False
            
            publish_setpoint(x, y, z, yaw)
            rate.sleep()
            
            distance = math.sqrt((drone_pos['x'] - x)**2 + (drone_pos['y'] - y)**2 + (drone_pos['z'] - z)**2)
            
            if distance < 0.8:
                # Зависаем
                hover_start = time.time()
                while time.time() - hover_start < HOVER_TIME:
                    publish_setpoint(x, y, z, yaw)
                    rate.sleep()
                break
    
    return True

# ========== MAIN ==========
rospy.loginfo("=" * 70)
rospy.loginfo("AUTONOMOUS DATA COLLECTION FLIGHT")
rospy.loginfo("=" * 70)
rospy.loginfo(f"Save directory: {SAVE_DIR}")
rospy.loginfo(f"Field: {GRID_SIZE_X}m x {GRID_SIZE_Y}m")
rospy.loginfo(f"Altitude: {MIN_ALTITUDE}m - {MAX_ALTITUDE}m")
rospy.loginfo("=" * 70)

Connect()

Takeoff(MIN_ALTITUDE)

waypoints = GenerateWaypoints()

rospy.loginfo("\nStarting autonomous flight...\n")
success = FlyMission(waypoints)

if success:
    Land()

rospy.loginfo("\n" + "=" * 70)
rospy.loginfo("MISSION COMPLETE")
rospy.loginfo(f"Total images: {image_count}")
rospy.loginfo(f"Saved to: {SAVE_DIR}")
rospy.loginfo("=" * 70)

rospy.sleep(5)
