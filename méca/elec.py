import Part
import FreeCAD, FreeCADGui
from FreeCAD import Vector, Rotation
App = FreeCAD
Gui = FreeCADGui

def debug(obj):
    FreeCAD.Console.PrintError('%r :"%s" %r\n' % (obj, obj.__doc__, dir(obj)))


class ElecComponent:
    """base for all electronic component"""

    def __init__(self, doc, comp, box, name='unknown', color=(1., 0., 0.)):
        """generic elec init"""
        # must be called at the end of the specific init

        self.name = name

        self.comp = doc.addObject("Part::Feature", name)
        self.comp.Shape = comp
        Gui.ActiveDocument.getObject(name).ShapeColor = color

        self.box = doc.addObject("Part::Feature", name + '_box')
        self.box.Shape = box

        self.pl = FreeCAD.Placement()

        Gui.ActiveDocument.getObject(name + "_box").Visibility = False

    def translate(self, vect):
        """translate component and box"""
        self.pl.Base = vect

        self.comp.Placement = self.pl
        self.box.Placement = self.pl

    def rotate(self, vect, angle):
        """rotate component and box"""
        self.pl.Rotation = Rotation(vect, angle)

        self.comp.Placement = self.pl
        self.box.Placement = self.pl

    def __getitem__(self, item):
        """return dimension of the board"""
        return self.data[item]

    def envelop(self):
        """return full collision box"""
        return self.box


class Arduino(ElecComponent):
    """Arduino Pro mini"""
    def __init__(self, doc, name='arduino'):
        data = {
            'x': 18., # mm
            'y': 2., # mm
            'z': 33., # mm
        }
        self.data = data
        col_box = {
            'x': 16., # mm
            'y': 4., # mm
            'z': 31., # mm (9 = rs connector)
        }
        self.col_box = col_box

        # make board
        comp = Part.makeBox(data['x'], data['y'], data['z'])
        comp.translate(Vector(-data['x'] / 2, -data['y'] / 2, -data['z'] / 2))
        comp = comp.fuse(comp)

        # make its collision box
        box = Part.makeBox(col_box['x'], col_box['y'] + 10, col_box['z'] + 9.)   # 9 mm rs connector
        box.translate(Vector(-col_box['x'] / 2, -col_box['y'] / 2, -col_box['z'] / 2 - 9))
        box = box.fuse(comp)

        ElecComponent.__init__(self, doc, comp, box, name, (0., 0.67, 0.))

class ConnectCard(ElecComponent):
    """connection card"""
    def __init__(self, doc, name='connection_card'):
        data = {
            'x': 21., # mm
            'y': 2., # mm
            'z': 64., # mm
        }
        self.data = data
        col_box = {
            'x': 17., # mm
            'y': 2., # mm
            'z': 62., # mm
        }
        self.col_box = col_box

        # make board
        comp = Part.makeBox(data['x'], data['y'], data['z'])
        comp.translate(Vector(-data['x'] / 2, -data['y'] / 2, -data['z'] / 2))
        comp = comp.fuse(comp)

        # make its collision box
        box = Part.makeBox(col_box['x'], col_box['y'] + 10, col_box['z'])
        box.translate(Vector(-col_box['x'] / 2, -col_box['y'] / 2, -col_box['z'] / 2))
        box = box.fuse(comp)

        ElecComponent.__init__(self, doc, comp, box, name, (0., 0.67, 0.))

class Mpu(ElecComponent):
    """MPU board"""
    def __init__(self, doc, name='MPU'):
        data = {
            'x': 20., # mm
            'y': 2., # mm
            'z': 20., # mm
        }
        self.data = data
        col_box = {
            'x': 18., # mm
            'y': 4., # mm
            'z': 18., # mm
        }
        self.col_box = col_box

        # make board
        comp = Part.makeBox(data['x'], data['y'], data['z'])
        comp.translate(Vector(-data['x'] / 2, -data['y'] / 2, -data['z'] / 2))
        comp = comp.fuse(comp)

        # make its collision box
        box = Part.makeBox(col_box['x'], col_box['y'], col_box['z'])
        box.translate(Vector(-col_box['x'] / 2, -col_box['y'] / 2 + 1, -col_box['z'] / 2))
        box = box.fuse(comp)

        ElecComponent.__init__(self, doc, comp, box, name, (0., 0.67, 0.))

class Pile9V(ElecComponent):
    """pile 9V"""
    def __init__(self, doc, name='pile9V'):
        data = {
            'x': 25.5, # mm
            'y': 16.5, # mm
            'z': 52.6, # mm
        }
        self.data = data
        col_box = {
            'x': 25.5, # mm
            'y': 16.5, # mm
            'z': 52.6, # mm
        }
        self.col_box = col_box

        # make board
        comp = Part.makeBox(data['x'], data['y'], data['z'])
        comp.translate(Vector(-data['x'] / 2, -data['y'] / 2, -data['z'] / 2))
        comp = comp.fuse(comp)

        # make its collision box
        box = Part.makeBox(col_box['x'], col_box['y'], col_box['z'] + 2)
        box.translate(Vector(-col_box['x'] / 2, -col_box['y'] / 2, -col_box['z'] / 2))
        box = box.fuse(comp)

        ElecComponent.__init__(self, doc, comp, box, name, (0., 0.33, 1.))

class Switch(ElecComponent):
    """switch contactor"""
    def __init__(self, doc, name='switch'):
        data = {
            'x': 12., # mm
            'y': 6., # mm
            'z': 20., # mm
        }
        self.data = data
        col_box = {
            'x': 14., # mm
            'y': 6., # mm
            'z': 20., # mm
        }
        self.col_box = col_box

        # make board
        comp = Part.makeBox(data['x'], data['y'], data['z'])
        comp.translate(Vector(-data['x'] / 2, -data['y'] / 2, -data['z'] / 2))
        comp = comp.fuse(comp)

        # make its collision box
        box = Part.makeBox(col_box['x'], col_box['y'], col_box['z'])
        box.translate(Vector(-col_box['x'] / 2, -col_box['y'] / 2, -col_box['z'] / 2))
        box = box.fuse(comp)

        ElecComponent.__init__(self, doc, comp, box, name, (0., 0., 0.))

class Servo(ElecComponent):
    """servo actuator"""
    def __init__(self, doc, name='servo'):
        data = {
            'x': 23., # mm
            'y': 12.5, # mm
            'z': 23., # mm
        }
        self.data = data
        col_box = {
            'x': 23., # mm
            'y': 12.5, # mm
            'z': 23., # mm
        }
        self.col_box = col_box

        # make board
        comp = Part.makeBox(data['x'], data['y'], data['z'])
        comp.translate(Vector(-data['x'] / 2, -data['y'] / 2, -data['z'] / 2))
        comp = comp.fuse(comp)

        # make its collision box
        box = Part.makeBox(col_box['x'], col_box['y'], col_box['z'])
        box.translate(Vector(-col_box['x'] / 2, -col_box['y'] / 2, -col_box['z'] / 2))
        box = box.fuse(comp)

        ElecComponent.__init__(self, doc, comp, box, name, (0., 0., 0.))

