import math
import numpy as np
from geometry_msgs.msg import Point

def normalize_angle(angle):
    """Normalisasi sudut ke range [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def distance(p1, p2):
    """Jarak Euclidean antara dua Point"""
    return math.hypot(p1.x - p2.x, p1.y - p2.y)

def angle_between_points(origin, target):
    """Sudut dari origin ke target (dalam radian)"""
    return math.atan2(target.y - origin.y, target.x - origin.x)

def rotate_point(point, angle):
    """Rotasi point (x,y) sebesar angle (radian)"""
    x = point.x * math.cos(angle) - point.y * math.sin(angle)
    y = point.x * math.sin(angle) + point.y * math.cos(angle)
    return Point(x=x, y=y, z=0.0)

def point_to_list(point):
    return [point.x, point.y]