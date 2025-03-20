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
