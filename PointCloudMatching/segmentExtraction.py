import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import random
import math

class point:
    def __init__(self, _x = 0, _y = 0):
        self.x = _x
        self.y = _y
    
    def __add__(self, a):
        return point(self.x + a.x, self.y + a.y)
    def __sub__(self, a):
        return point(self.x - a.x, self.y - a.y)
    
    def __mul__(self, a):
        return point(self.x * a, self.y * a)    
    def __div__(self, a):
        return point(self.x / a, self.y / a)
    
    def __mul__(self, a):
        return self.x * a.x + self.y * a.y
    def __xor__(self, a):
        return self.x * a.y - self.y * a.x
    
    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
    
    def unit(self):
        return self / self.length()

    def __str__(self):
        return f"Point({self.x}, {self.y})"

def read_scan_points(file):
    fp = open(file, 'r')
    points = []
    for data in fp:
        data = data.replace('\n', '')
        data = data.split(' ')
        x = float(data[0])
        y = float(data[1])
        points.append([x, y])
    fp.close()
    return points


def plot_points(points1, points2, segments, block):
    # if not hasattr(plot_points, "first_call"):
    #     plot_points.first_call = True

    points1 = toArray(points1)
    points2 = toArray(points2)

    x1, y1 = zip(*points1)
    x2, y2 = zip(*points2)
    plt.clf()
    plt.xlim(-10.0, 15.0)
    plt.ylim(-10.0, 15.0)
    plt.scatter(x2, y2, color='blue', label='Target points', s=20)
    plt.scatter(x1, y1, color='red', label='Source points', s=10)
    for segment in segments:
        point1, point2 = (segment[0].x, segment[0].y), (segment[1].x, segment[1].y)
        plt.plot([point1[0], point2[0]], [point1[1], point2[1]], color='green', linewidth=2)
    plt.legend()
    plt.grid(True)
    plt.title("SegmentExtraction")
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend(loc='upper left', fontsize=12)
    plt.show(block=block)
    plt.draw()
    plt.pause(0.2)

def toPoints(lst):
    result = []
    for i in range(len(lst)):
        result.append(point(lst[i][0], lst[i][1]))
    return result.copy()

def toArray(pts):
    result = []
    for i in range(len(pts)):
        result.append([pts[i].x, pts[i].y])
    return result.copy()

def distToLine(p, st, ed):
    return abs(st - p ^ ed - p) / (st - ed).length()

def distToSegment(p, st, ed):
    if (ed - st) * (p - st) < 0:
        return (p - st).length()
    if (st - ed) * (p - ed) < 0:
        return (p - ed).length()
    return abs(st - p ^ ed - p) / (st - ed).length()

def nearestPoints(points, st, ed, delta):
    result = []
    for p in points:
        if distToLine(p, st, ed) < delta:
            result.append(p)
    return result

def leastSquaresRegression(points):
    x = [p.x for p in points]
    y = [p.y for p in points]
    
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    
    numerator = np.sum((x - x_mean) * (y - y_mean))
    denominator = np.sum((x - x_mean)**2)
    
    slope = numerator / denominator
    intercept = y_mean - slope * x_mean
    
    def line_func(x_val):
        return slope * x_val + intercept
    
    return slope, intercept, line_func

def segmentExtraction(points):
    step = 100
    minPoints = 50
    eps = 1e-1

    result = []
    n = len(points)
    while step > 0:
        step -= 1
        i = random.randint(0, n - 1)
        j = random.randint(0, n - 1)
        linePoints = nearestPoints(points, points[i], points[j], eps)
        if len(linePoints) > minPoints:
            # y = a * x + b
            a, b, f = leastSquaresRegression(linePoints)
            result.append([point(-10, f(-10)), point(15, f(15))])
    return result


scan_points1 = toPoints(read_scan_points('./data/scan_1.txt'))
scan_points2 = toPoints(read_scan_points('./data/scan_2.txt'))


segments = segmentExtraction(scan_points1)

plot_points(scan_points1, scan_points2, segments, True)

