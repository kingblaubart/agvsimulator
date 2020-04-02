from Point import Point
import math as m
import numpy as np
import sympy as sy
from scipy import signal

eventqueue = None
dt = None
ct = None
pt = None
fps = None
carList = []
data = []
vis_data = []
debug_list = []
collision = None
coll_det_freq = None
car_count = None
k_p = None
k_d = None
last_timestamp = None
statespace = None
holonom = None
last_get_data = 0


# some setter methods
def set_eventqueue(eq):
    global eventqueue
    eventqueue = eq


def set_dt(t):
    global dt
    dt = t


def set_ct(t):
    global ct
    ct = t


def set_pt(t):
    global pt
    pt = t


def set_fps(framerate):
    global fps
    fps = framerate


def set_collision(c):
    global collision
    collision = c


def set_coll_det_freq(cdf):
    global coll_det_freq
    coll_det_freq = cdf


def set_carcount(c):
    global car_count
    car_count = c


def set_k_p(kp):
    global k_p
    k_p = kp


def set_k_d(kd):
    global k_d
    k_d = kd


def set_statespace(ss):
    global statespace
    statespace = ss


def set_holonom(h):
    global holonom
    holonom = h


# some methods used my multiple classes
def angle(p1: Point, p2: Point):
    phi = m.atan2(p2.y - p1.y, p2.x - p1.x)
    return phi


def goodangle(p1, p2):
    phi = m.atan2(p2.imag - p1.imag, p2.real - p1.real)
    return phi


def distance(p1: Point, p2: Point):
    x = p2.x - p1.x
    y = p2.y - p1.y
    d = np.linalg.norm([x, y])
    return d


def dist(a, b):
    a = np.array(a)
    b = np.array(b)
    return np.linalg.norm(a-b)
