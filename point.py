import math

class point:
    def __init__(self, _x = 0., _y = 0.):
        self.x = _x
        self.y = _y
    
    def __add__(self, a):
        return point(self.x + a.x, self.y + a.y)
    def __sub__(self, a):
        return point(self.x - a.x, self.y - a.y)
    
    def __mul__(self, a):
        if isinstance(a, point):
            return self.x * a.x + self.y * a.y
        return point(self.x * a, self.y * a)
    
    def __truediv__(self, a):
        return point(self.x / a, self.y / a)
    
    def __xor__(self, a):
        return self.x * a.y - self.y * a.x
    
    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
    
    def unit(self):
        if self.length() < 1e-9:
            return self
        return self / self.length()

    def __str__(self):
        return f"Point({self.x}, {self.y})"


# distance from point a to line bc
def dist_to_line(a, b, c):
    if (c - b).length() < 1e-9:
        return (b - a).length()
    return abs((c - a) ^ (b - a)) / (c - b).length()

a = point(0, 0)
b = point(0, 1)
c = point(1, 0)


VFOV = 40 * math.pi / 180
IMV = 9
IMH = 16
HFOV = (math.atan(math.tan(VFOV / 2) / IMV * IMH) * 2) / math.pi * 180

print(HFOV)
