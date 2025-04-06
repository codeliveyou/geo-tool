from point import point
import math

earthr = 6378000
R2D = 180 / math.pi
D2R = math.pi / 180

def lla_to_ned(lat0, lon0, lat, lon):
    return point((lat - lat0) * D2R * earthr, (lon - lon0) * D2R * (earthr * math.cos(lat0 * D2R)))

def ned_to_lla(lat0, lon0, n, e):
    return point(n / earthr * R2D + lat0, e / (earthr *  math.cos(lat0 * D2R)) * R2D + lon0)

def global_to_map(ga, gb, gc, ma, mb, mc, gx, gy):
    na = lla_to_ned(ga.x, ga.y, ga.x, ga.y)
    nb = lla_to_ned(ga.x, ga.y, gb.x, gb.y)
    nc = lla_to_ned(ga.x, ga.y, gc.x, gc.y)
    nd = lla_to_ned(ga.x, ga.y, gx, gy)

    x = (nb - na).unit() * (nd - na) / (nb - na).length()
    y = (nc - na).unit() * (nd - na) / (nc - na).length()
    md = (mb - ma) * x + (mc - ma) * y
    return [md.x, md.y]

def map_to_global(ma, mb, mc, ga, gb, gc, mx, my):
    md = point(mx, my)
    x = (mb - ma).unit() * (md - ma) / (mb - ma).length()
    y = (mc - ma).unit() * (md - ma) / (mc - ma).length()

    na = lla_to_ned(ga.x, ga.y, ga.x, ga.y)
    nb = lla_to_ned(ga.x, ga.y, gb.x, gb.y)
    nc = lla_to_ned(ga.x, ga.y, gc.x, gc.y)
    nd = (nb - na) * x + (nc - na) * y

    gd = ned_to_lla(ga.x, ga.y, nd.x, nd.y)

    return [gd.x, gd.y]

ga = point(39.053078, 125.769001)
gb = point(39.053301, 125.768969)
gc = point(39.053117, 125.769449)

ma = point(0, 0)
mb = point(0, 599)
mc = point(899, 0)

print(global_to_map(ga, gb, gc, ma, mb, mc, 39.053228, 125.769138))

print(map_to_global(ma, mb, mc, ga, gb, gc, 316.35, 368.36))
