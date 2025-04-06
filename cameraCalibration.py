import math

def deg2Rad(a):
    return a / 180. * math.pi

def rad2Deg(a):
    return a / math.pi * 180

imh = 720
imw = 1280

vfov = 22.5
hfov = 38.9

# hfov = rad2Deg(2 * math.atan(math.tan(deg2Rad(vfov) / 2) / imh * imw))
vfov = rad2Deg(2 * math.atan(math.tan(deg2Rad(hfov) / 2) / imw * imh))
print(f"vfov = {vfov}, hfov = {hfov}")


f = 6

sh = f * math.tan(deg2Rad(vfov) / 2) * 2
sw = f * math.tan(deg2Rad(hfov) / 2) * 2

print(math.tan(deg2Rad(hfov) / 2) / math.tan(deg2Rad(vfov) / 2), 1280 / 720)

print(f"F = {f}, Senser H = {sh}, W = {sw}")

sh = 7.2
sw = 12.8

f = sh / math.tan(deg2Rad(vfov) / 2) / 2
print(f"F = {f}, Senser H = {sh}, W = {sw}")
f = sw / math.tan(deg2Rad(hfov) / 2) / 2
print(f"F = {f}")

# Sensor Size
# For a 1/2.7" sensor:
# - Width: Typically around 5.37 mm
# - Height: Typically around 4.04 mm

# Focal Length
# - Shortest Focal Length (Wide): 4.5 mm

# Field of View
# - Horizontal FOV: 61.87°
# - Vertical FOV: 37.4°

print(5.37 / 4.04, math.tan(deg2Rad(61.87) / 2) / math.tan(deg2Rad(37.4) / 2))

f = 4.5
vfov = 37.4
hfov = 61.87

sh = f * math.tan(deg2Rad(vfov) / 2) * 2
sw = f * math.tan(deg2Rad(hfov) / 2) * 2

print(f"F = {f}, Senser H = {sh}, W = {sw}")

alpha = deg2Rad(37.4)
beta = deg2Rad(61.87)

print(math.sqrt(3) * math.tan(beta / 2) * 2 * (math.tan(math.pi / 6 + alpha / 2) - math.tan(math.pi / 6 - alpha / 2)))

