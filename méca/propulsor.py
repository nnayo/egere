import Part
from FreeCAD import Vector

# propulseur Pro 24 1G
propulsor_data = {
    'len': 69.3, # mm
    'diameter': 23.8, # mm
    'hold diameter': 25.4, # mm
    'hold thick': 6.4, # mm
    'tuyere diameter': 8., # mm
    'tuyere len': 4., # mm
}

guide_data = {
    'thick': 2., # mm thickness of the guide wall
}


def propulsor():
    """propulsor"""
    prop = propulsor_data

    # approximate the prop
    body = Part.makeCylinder(prop['diameter'] / 2, prop['len'])
    hold = Part.makeCylinder(prop['hold diameter'] / 2, prop['hold thick'])
    hold.translate(Vector(0, 0, -prop['hold thick']))
    tuyere = Part.makeCylinder(prop['tuyere diameter'] / 2, prop['tuyere len'])
    tuyere.translate(Vector(0, 0, -prop['hold thick'] - prop['tuyere len']))

    obj = body.fuse(hold)
    obj = obj.fuse(tuyere)

    return obj


def guide():
    """guide"""
    prop = propulsor_data

    # make the guide
    propu = Part.makeCylinder(prop['diameter'] / 2, prop['len'])

    obj = Part.makeCylinder(prop['diameter'] / 2 + guide_data['thick'], prop['len'])
    obj = obj.cut(propu)

    return obj

