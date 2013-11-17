import Part
from FreeCAD import Vector

# arduino
arduino_data = {
    'x': 18., # mm
    'y': 2., # mm
    'z': 33. + 9., # mm (9 = rs connector)
}

# connection card
connect_card_data = {
    'x': 21., # mm
    'y': 3., # mm
    'z': 64., # mm
}

# MPU
mpu_data = {
    'x': 20., # mm
    'y': 2., # mm
    'z': 20., # mm
}

# switch
switch_data = {
    'x': 15., # mm
    'y': 6., # mm
    'z': 20., # mm
}

# servo
servo_data = {
    'x': 23., # mm
    'y': 12.5, # mm
    'z': 23., # mm
}

# pile 9V
pile_9v_data = {
    'x': 25.5, # mm
    'y': 16.5, # mm
    'z': 52.6, # mm
}


def arduino():
    """make an Arduino Pro mini"""

    ad = arduino_data
    # make arduino
    comp = Part.makeBox(ad['x'], ad['y'], ad['z'])
    comp.translate(Vector(-ad['x'] / 2, -ad['y'] / 2, -ad['z'] / 2))

    return comp


def connect_card():
    """make an connection card"""

    # make card
    comp = Part.makeBox(connect_card_data['x'], connect_card_data['y'], connect_card_data['z'])
    comp.translate(Vector(-connect_card_data['x'] / 2, -connect_card_data['y'] / 2, -connect_card_data['z'] / 2))

    #comp.rotate(Vector(0, 0, 0), Vector(0, 0, 1), 180)
    return comp


def mpu():
    """make a MPU card"""

    # make card
    comp = Part.makeBox(mpu_data['x'], mpu_data['y'], mpu_data['z'])
    comp.translate(Vector(-mpu_data['x'] / 2, -mpu_data['y'] / 2, -mpu_data['z'] / 2))

    return comp


def pile_9v():
    """make a 9V pile"""

    # make pile
    comp = Part.makeBox(pile_9v_data['x'], pile_9v_data['y'], pile_9v_data['z'])
    comp.translate(Vector(-pile_9v_data['x'] / 2, -pile_9v_data['y'] / 2, -pile_9v_data['z'] / 2))

    return comp


def switch():
    """switch"""

    # make switch
    comp = Part.makeBox(switch_data['x'], switch_data['y'], switch_data['z'])
    comp.translate(Vector(-switch_data['x'] / 2, -switch_data['y'] / 2, -switch_data['z'] / 2))

    return comp


def servo():
    """servo"""

    # make switch
    comp = Part.makeBox(servo_data['x'], servo_data['y'], servo_data['z'])
    comp.translate(Vector(-servo_data['x'] / 2, -servo_data['y'] / 2, -servo_data['z'] / 2))

    return comp


