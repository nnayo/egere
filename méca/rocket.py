import FreeCAD
from FreeCAD import Vector

import meca
import elec


def elec_draw(doc, tube):
    offset_elec = tube['len'] + 5

    swit = elec.Switch(doc)
    swit.translate(Vector(-swit['x'] / 2 + tube['int diameter'] / 2 - 1, 0, offset_elec - swit['z'] / 2))

    servo_cone = elec.Servo(doc, 'servo_cone')
    servo_cone.translate(Vector(0, 0, offset_elec + servo_cone['z'] / 2))

    offset_elec += servo_cone['z'] + 5

    pile = elec.Pile9V(doc)
    pile.translate(Vector(0, 0, offset_elec + pile['z'] / 2))

    arduino = elec.Arduino(doc)
    arduino.translate(Vector(0, arduino['y'] / 2 + 2 + pile['y'] / 2, offset_elec + arduino['z'] / 2))

    mpu = elec.Mpu(doc)
    mpu.translate(Vector(0, arduino['y'] / 2 + 2 + pile['y'] / 2, offset_elec + arduino['z'] + mpu['z'] / 2 + 2))

    connect = elec.ConnectCard(doc)
    connect.rotate(Vector(0, 0, 1), 180)
    connect.translate(Vector(0, -connect['y'] / 2 - 2 - pile['y'] / 2, offset_elec + connect['z'] / 2))

    #obj = Part.makeCylinder(200, 1)
    #obj.translate(Vector(0, 0, rd.tube['len']))
    #doc.addObject("Part::Feature", 'bottom_minut_zone').Shape = obj
    #FreeCAD.Gui.ActiveDocument.getObject("bottom_minut_zone").Visibility = False

    servo_aero = elec.Servo(doc, 'servo_aero')
    servo_aero.translate(Vector(0, 0, tube['len'] / 2 + servo_aero['z']))

    return (swit, servo_cone, pile, arduino, mpu, connect)

def tube_draw(doc):
    """draw the tube"""
    # tube
    tube = meca.Tube(doc)

    # cone
    cone = None
    #cone = meca.Cone(doc)
    #cone.translate(Vector(0, 0, tube['len']))

    #helix = meca.Helix(doc)
    #helix.translate(Vector(0, 0, tube['len']))

    # propulsor
    propu = meca.Propulsor(doc)
    propu.translate(Vector(0, 0, tube['len'] / 2))

    # isolator
    isol = meca.Isolator(doc, propu)
    isol.translate(Vector(0, 0, tube['len'] / 2))

    # propulsor bague
    bague = meca.BaguePropu(doc, tube, isol)
    bague.translate(Vector(0, 0, tube['len'] / 2))

    # aero-brakes
    aero = []
    aero.append(meca.AeroBrake(doc, tube, 'aero_brake0'))
    aero.append(meca.AeroBrake(doc, tube, 'aero_brake1'))
    aero.append(meca.AeroBrake(doc, tube, 'aero_brake2'))

    aero[0].rotate(Vector(0, 0, 1), 0)
    aero[1].rotate(Vector(0, 0, 1), 120)
    aero[2].rotate(Vector(0, 0, 1), 240)

    aero[0].translate(Vector(0, 0, tube['len'] / 2 + bague['len'] + 5))
    aero[1].translate(Vector(0, 0, tube['len'] / 2 + bague['len'] + 5))
    aero[2].translate(Vector(0, 0, tube['len'] / 2 + bague['len'] + 5))

    # aero-brake clutch
    aero_clutch = meca.AeroClutch(doc, tube, 'aero_clutch')
    aero_clutch.translate(Vector(0, 0, tube['len'] / 2 + bague['len'] + 5))

    return tube, cone

def integ_draw(doc, tube, cone, elec_comps):
    # integration bague
    bague = meca.Bague(doc)
    bague.translate(Vector(0, 0, tube['len'] - bague['len lo'] - bague['len hi']))

    # integration
    integ1 = meca.Integ1(doc)
    integ1.translate(Vector(0, 0, tube['len'] - bague['len hi']))
    integ2 = meca.Integ2(doc)
    integ2.translate(Vector(0, 0, tube['len'] - bague['len hi']))

    for ec in elec_comps:
        integ1.cut(ec.envelop().Shape)
        integ2.cut(ec.envelop().Shape)

    integ1.cut(cone.envelop().Shape)

def main(doc):
    tube, cone = tube_draw(doc)
    #elec_comps = elec_draw(doc, tube)
    #integ_draw(doc, tube, cone, elec_comps)

    FreeCAD.Gui.activeDocument().activeView().viewAxometric()
    FreeCAD.Gui.SendMsgToActiveView("ViewFit")


if __name__ == "__main__":
    doc = FreeCAD.activeDocument()
    if doc == None:
        doc = FreeCAD.newDocument('egere')

    main(doc)
