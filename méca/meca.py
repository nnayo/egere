import Part
import FreeCAD, FreeCADGui
from FreeCAD import Vector, Matrix, Rotation
Gui = FreeCADGui


def debug(obj):
    FreeCAD.Console.PrintError('%r :"%s" %r\n' % (obj, obj.__doc__, dir(obj)))



# ouies
ouie_data = {
    'length': 250., # mm
    'radius': 12., # mm
    'offset z': 250. # mm
}

# decoupes aerofreins
aero_data = {
    'side': 12., # mm
    'offset z': 500. # mm
}


class MecaComponent:
    """base for all mechanical component"""

    def __init__(self, doc, comp, name='unknown', color=(1., 0., 0.)):
        """generic meca init"""
        # must be called at the end of the specific init

        self.name = name

        self.pl = FreeCAD.Placement()

        self.comp = doc.addObject("Part::Feature", name)
        self.comp.Shape = comp
        Gui.ActiveDocument.getObject(name).ShapeColor = color

    def translate(self, vect):
        """translate component and box"""
        self.pl.Base = vect

        self.comp.Placement = self.pl

    def rotate(self, vect, angle):
        """rotate component and box"""
        self.pl.Rotation = Rotation(vect, angle)

        self.comp.Placement = self.pl

    def __getitem__(self, item):
        """return dimension of the board"""
        return self.data[item]

    def cut(self, shape):
        """overload the cut method"""
        self.comp.Shape = self.comp.Shape.cut(shape)

    def envelop(self):
        return self.comp


class Tube(MecaComponent):
    """tube"""
    def __init__(self, doc, name='tube'):
        self.data = {
            'len': 1000., # mm
            'int diameter': 36., # mm
            'thick': 1., # mm
        }

        int_diam = self.data['int diameter']
        length = self.data['len']
        ext_diam = int_diam + 2 * self.data['thick']

        int_tube = Part.makeCylinder(int_diam / 2, length)
        ext_tube = Part.makeCylinder(ext_diam / 2, length)

        comp = ext_tube.cut(int_tube)

        MecaComponent.__init__(self, doc, comp, name, (0., 0., 0.))


class Cone(MecaComponent):
    """cone"""
    def __init__(self, doc, name='cone'):
        self.data = {
            'len lo': 70., # mm
            'len up': 120., # mm
            'int diameter': 36., # mm
            'thick': 1., # mm
        }

        int_diam = self.data['int diameter']
        len_up = self.data['len up']
        len_lo = self.data['len lo']
        ext_diam = int_diam + 2 * self.data['thick']

        int_cone = Part.makeSphere(int_diam / 2)
        ext_cone = Part.makeSphere(ext_diam / 2)

        box = Part.makeBox(ext_diam, ext_diam, ext_diam)
        box.translate(Vector(-ext_diam / 2, -ext_diam / 2, 0))

        cone_up = ext_cone.cut(int_cone)
        cone_up = cone_up.common(box)

        matrix = Matrix()
        matrix.scale(1., 1., len_up / (ext_diam / 2))
        cone_up = cone_up.transformGeometry(matrix)
        cone_up.translate(Vector(0, 0, len_lo))
        cone_up = cone_up.fuse(cone_up)

        # blocking thread (pas de vis de blocage)
#        radius = self.data['int diameter'] / 2
#
#        helix = Part.makeHelix(4., 20., radius)
#
#        p0 = (radius, 0, 0)
#        p1 = (radius, 0, 3)
#        p2 = (radius - 1, 0, 2)
#        p3 = (radius - 1, 0, 1)
#
#        e0 = Part.makeLine(p0, p1)
#        e1 = Part.makeLine(p1, p2)
#        e2 = Part.makeLine(p2, p3)
#        e3 = Part.makeLine(p3, p0)
#        section = Part.Wire([e0, e1, e2, e3])
#        helix = Part.Wire(helix).makePipeShell([section], 1, 1)
#        helix.translate(Vector(0, 0, len_lo - 20))

        int_tube = Part.makeCylinder(int_diam / 2, len_lo)
        ext_tube = Part.makeCylinder(ext_diam / 2, len_lo)

        cone_lo = ext_tube.cut(int_tube)
#        cone_lo = cone_lo.fuse(helix)

        #comp = cone_up.fuse(cone_lo) # BUG: this fusion fails!!!
        comp = cone_lo

        MecaComponent.__init__(self, doc, comp, name, (1., 1., 0.))


class Helix(MecaComponent):
    """helix"""
    def __init__(self, doc, name='helix'):
        self.data = {
            'len lo': 70., # mm
            'len up': 120., # mm
            'int diameter': 36., # mm
            'thick': 1., # mm
        }

        len_lo = self.data['len lo']

        # blocking thread (pas de vis de blocage)
        radius = self.data['int diameter'] / 2

        helix = Part.makeHelix(4., 16., radius)

        p0 = (radius, 0, 0)
        p1 = (radius, 0, 3)
        p2 = (radius - 1, 0, 2)
        p3 = (radius - 1, 0, 1)

        e0 = Part.makeLine(p0, p1)
        e1 = Part.makeLine(p1, p2)
        e2 = Part.makeLine(p2, p3)
        e3 = Part.makeLine(p3, p0)
        section = Part.Wire([e0, e1, e2, e3])
        helix = Part.Wire(helix).makePipeShell([section], 1, 1)
        helix.translate(Vector(0, 0, len_lo - 20))

        helix = helix.fuse(helix)

        comp = helix

        MecaComponent.__init__(self, doc, comp, name, (1., 1., 0.))


class Bague(MecaComponent):
    """bague"""
    def __init__(self, doc, name='bague'):
        self.data = {
            'len lo': 40., # mm
            'len hi': 20., # mm
            'ext diameter': 36., # mm
            'int diameter': 30., # mm
        }

        int_diam = self.data['int diameter']
        ext_diam = self.data['ext diameter']
        len_hi = self.data['len hi']
        len_lo = self.data['len lo']

        cone_lo = Part.makeCone(ext_diam / 2, int_diam / 2, len_lo)

        cone_hi = Part.makeCone(int_diam / 2, ext_diam / 2, len_hi)
        cone_hi.translate(Vector(0, 0, len_lo))

        comp = Part.makeCylinder(ext_diam / 2, len_lo + len_hi).cut(cone_hi).cut(cone_lo)

        MecaComponent.__init__(self, doc, comp, name, (1., 1., 0.))

class Integ1(MecaComponent):
    """integ first halp"""
    def __init__(self, doc, name='integ1'):
        self.data = {
            'len lo': 20., # mm
            'len hi': 100., # mm
            'ext diameter': 36., # mm
            'int diameter': 30., # mm
        }

        int_diam = self.data['int diameter']
        ext_diam = self.data['ext diameter']
        len_hi = self.data['len hi']
        len_lo = self.data['len lo']

        cone = Part.makeCone(int_diam / 2, ext_diam / 2, len_lo)

        trunk = Part.makeCylinder(ext_diam / 2, len_hi)
        trunk.translate(Vector(0, 0, len_lo))

        comp = cone.fuse(trunk)

        cutter = Part.makeBox(ext_diam, ext_diam, len_hi + len_lo)
        cutter.translate(Vector(0, -ext_diam / 2, 0))

        comp = comp.common(cutter)

        MecaComponent.__init__(self, doc, comp, name, (1., 1., 0.))

class Integ2(MecaComponent):
    """integ second half"""
    def __init__(self, doc, name='integ2'):
        self.data = {
            'len lo': 20., # mm
            'len hi': 100., # mm
            'ext diameter': 36., # mm
            'int diameter': 30., # mm
        }

        int_diam = self.data['int diameter']
        ext_diam = self.data['ext diameter']
        len_hi = self.data['len hi']
        len_lo = self.data['len lo']

        cone = Part.makeCone(int_diam / 2, ext_diam / 2, len_lo)

        trunk = Part.makeCylinder(ext_diam / 2, len_hi)
        trunk.translate(Vector(0, 0, len_lo))

        comp = cone.fuse(trunk)

        cutter = Part.makeBox(ext_diam, ext_diam, len_hi + len_lo)
        cutter.translate(Vector(0, -ext_diam / 2, 0))
        cutter.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 180)

        comp = comp.common(cutter)

        MecaComponent.__init__(self, doc, comp, name, (1., 1., 0.))

class Propulsor(MecaComponent):
    """propulsor Cesaroni Pro 24 1G"""

    def __init__(self, doc, name='Pro_24_1G'):
        self.data = {
            'len': 69.3, # mm
            'diameter': 23.8, # mm
            'hold diameter': 25.4, # mm
            'hold thick': 6.4, # mm
            'tuyere diameter': 8., # mm
            'tuyere len': 4., # mm
        }


        # approximate the prop
        prop = self.data
        body = Part.makeCylinder(prop['diameter'] / 2, prop['len'])
        hold = Part.makeCylinder(prop['hold diameter'] / 2, prop['hold thick'])
        hold.translate(Vector(0, 0, -prop['hold thick']))
        tuyere = Part.makeCylinder(prop['tuyere diameter'] / 2, prop['tuyere len'])
        tuyere.translate(Vector(0, 0, -prop['hold thick'] - prop['tuyere len']))

        comp = body.fuse(hold)
        comp = comp.fuse(tuyere)

        MecaComponent.__init__(self, doc, comp, name, (0.95, 1., 1.))


class Isolator(MecaComponent):
    """isolator for propulsor Cesaroni Pro 24 1G"""

    def __init__(self, doc, prop, name='isolator'):
        self.data = {
            'thick': 1., # mm
        }
        data = self.data
        data['len'] = prop['len'] + data['thick'] # mm
        data['diameter'] = prop['diameter'] + data['thick'] # mm

        propu = Part.makeCylinder(prop['diameter'] / 2, prop['len'])

        isol = Part.makeCylinder(data['diameter'] / 2, data['len'])
        comp = isol.cut(propu)

        MecaComponent.__init__(self, doc, comp, name, (0.66, 0.33, 0.))


class BaguePropu(MecaComponent):
    """bague holding the isolator in the tube"""

    def __init__(self, doc, tube, isol, name='bague_propu'):
        self.data = {
            'thick': 2., # mm
        }
        data = self.data
        data['len'] = isol['len'] + data['thick'] # mm
        data['int diameter'] = isol['diameter'] + data['thick'] # mm
        data['ext diameter'] = tube['int diameter'] # mm

        isola = Part.makeCylinder(isol['diameter'] / 2, isol['len'])

        ext = Part.makeCylinder(data['ext diameter'] / 2, data['len'])
        comp = ext.cut(isola)

        MecaComponent.__init__(self, doc, comp, name, (1., 1., 0.))


class AeroBrake(MecaComponent):
    """a single aero-brake"""

    def __init__(self, doc, tube, name='aero_brake'):
        self.data = {
            'len': 20., # mm
            'width' : 15., # mm
        }
        data = self.data
        data['thick'] = tube['thick'] # mm
        data['ext diameter'] = tube['int diameter'] + tube['thick'] # mm
        data['int diameter'] = tube['int diameter'] # mm

        # rebuild a tube
        int_part = Part.makeCylinder(data['int diameter'] / 2, data['len'])
        ext_part = Part.makeCylinder(data['ext diameter'] / 2, data['len'])
        part = ext_part.cut(int_part)

        shape = Part.makeBox(data['ext diameter'], data['width'], data['len'])
        shape.translate(Vector(-data['ext diameter'], -data['width'] / 2, 0))

        comp = part.common(shape)

        MecaComponent.__init__(self, doc, comp, name, (1., 1., 0.))
