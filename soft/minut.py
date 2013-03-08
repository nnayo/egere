"""
generate the eeprom image for egere minuterie frames
"""


from frame import *
from frame import Frame


CONE_SERVO = 0xaa
CONE_AERO = 0x55

SERVO_SAVE = 0x00

SERVO_OPEN_POS = 0x00
SERVO_CLOSE_POS = 0xff

TIME_OUT_SAVE = 0x00

I2C_SELF_ADDR = Frame.I2C_SELF_ADDR
T_ID = Frame.T_ID
CMD = Frame.CMD


# slots number
slots_nb = 2

slots = [
	#--------------------------------
	# slot #0 : reset
	[
		# set cone servo open position: -15deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, CONE_SERVO, SERVO_SAVE, SERVO_OPEN_POS, -15),

		# set cone servo closed position: +30deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, CONE_SERVO, SERVO_SAVE, SERVO_CLOSE_POS, 30),

		# set aero servo open position: -15deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, CONE_AERO, SERVO_SAVE, SERVO_OPEN_POS, -15),

		# set aero servo closed position: +30deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, CONE_AERO, SERVO_SAVE, SERVO_CLOSE_POS, 30),

		# set flight time-out: 12.5s
		minut_time_out(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, TIME_OUT_SAVE, 125),

		# set flight take-off detection threshold (10 * 10ms, 30 * 0.1G)
		take_off_thres(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, 10, 30),

		# set testing take-off detection threshold (200 * 10ms, 8 * 0.1G)
		#take_off_thres(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, 200, 8),
	],

	#--------------------------------
	# slot #1 : spare
	[
		no_cmde(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD),
	],
]

