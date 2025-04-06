from point import point

def coordinate_transformer(ga, gb, gc, ma, mb, mc, gx, gy):
    gd = point(gx, gy)
    x = (gb - ga).unit() * (gd - ga) / (gb - ga).length()
    y = (gc - ga).unit() * (gd - ga) / (gc - ga).length()
    md = (mb - ma) * x + (mc - ma) * y
    return [md.x, md.y]



ga = point(39.053078, 125.769001)
gb = point(39.053301, 125.768969)
gc = point(39.053117, 125.769449)

ma = point(0, 0)
mb = point(0, 599)
mc = point(899, 0)

print(coordinate_transformer(ga, gb, gc, ma, mb, mc, 39.053228771840985, 125.76913886850882))
