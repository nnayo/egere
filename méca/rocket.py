import FreeCAD
from FreeCAD import Vector
import Part

import rocket_data as rd
import tube
import propulsor
import elec


def prop_draw(doc):
    # create and position

    # propulsor
    propu = propulsor.propulsor()
    propu.translate(Vector(0, 0, rd.tube['len'] / 2))
    doc.addObject("Part::Feature", "propu").Shape = propu

    # guide
    guid = propulsor.guide()
    guid.translate(Vector(0, 0, rd.tube['len'] / 2))
    doc.addObject("Part::Feature", "guide").Shape = guid


def elec_draw(doc):
    offset_elec = rd.tube['len'] + 5

    # minut
    comp = elec.pile_9v()
    #comp.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 60)
    comp.translate(Vector(0, 0, offset_elec + elec.pile_9v_data['z'] / 2))
    doc.addObject("Part::Feature", 'pile_9v').Shape = comp

    comp = elec.arduino()
    #comp.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 60)
    comp.translate(Vector(0, elec.arduino_data['y'] / 2 + 2 + elec.pile_9v_data['y'] / 2, offset_elec + elec.arduino_data['z'] / 2))
    doc.addObject("Part::Feature", 'minuterie').Shape = comp

    comp = elec.mpu()
    comp.translate(Vector(0, elec.arduino_data['y'] / 2 + 2 + elec.pile_9v_data['y'] / 2, offset_elec + elec.arduino_data['z'] + elec.mpu_data['z'] / 2 + 2))
    #comp.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 60)
    doc.addObject("Part::Feature", 'mpu').Shape = comp

    comp = elec.connect_card()
    #comp.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 60)
    comp.translate(Vector(0, -elec.connect_card_data['y'] / 2 - 2 - elec.pile_9v_data['y'] / 2, offset_elec + elec.connect_card_data['z'] / 2))
    doc.addObject("Part::Feature", 'connection').Shape = comp

    comp = elec.servo()
    comp.translate(Vector(0, 0, rd.tube['len'] - elec.servo_data['z'] / 2))
    #comp.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 60)
    doc.addObject("Part::Feature", 'servo_cone').Shape = comp

    comp = elec.switch()
    comp.translate(Vector(-elec.switch_data['x'] / 2 + rd.tube['int diameter'] / 2, 0, rd.tube['len'] - elec.servo_data['z'] - elec.switch_data['z'] / 2))
    #comp.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 60)
    doc.addObject("Part::Feature", 'switch_cone').Shape = comp

    obj = Part.makeCylinder(200, 1)
    obj.translate(Vector(0, 0, rd.tube['len']))
    doc.addObject("Part::Feature", 'bottom_minut_zone').Shape = obj
    #FreeCAD.Gui.ActiveDocument.getObject("bottom_minut_zone").Visibility = False

    comp = elec.servo()
    comp.translate(Vector(0, 0, rd.tube['len'] / 2 + elec.servo_data['z']))
    #comp.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 60)
    doc.addObject("Part::Feature", 'servo_aero').Shape = comp


def tube_draw(doc):
    """draw the tube"""
    # tube
    comp = tube.tube()
    doc.addObject("Part::Feature", 'tube').Shape = comp
    FreeCAD.Gui.ActiveDocument.getObject("tube").Visibility = False



def main(doc):
    tube_draw(doc)
    prop_draw(doc)
    elec_draw(doc)
    #aero_draw(doc)
    #cone_draw(doc)

    FreeCAD.Gui.SendMsgToActiveView("ViewFit")


if __name__ == "__main__":
    doc = FreeCAD.activeDocument()
    if doc == None:
        doc = FreeCAD.newDocument()

    main(doc)
