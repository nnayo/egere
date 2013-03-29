"""
generate the eeprom image for egere minuterie frames
"""


from frame import *
from frame import Frame


SERVO_CONE = 0xc0
SERVO_AERO = 0xae

SERVO_SAVE = 0x5a
SERVO_READ = 0x4e

SERVO_OPEN_POS = 0x09
SERVO_CLOSE_POS = 0xc1

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
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_CONE, SERVO_SAVE, SERVO_OPEN_POS, -15),

		# set cone servo closed position: +30deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_CONE, SERVO_SAVE, SERVO_CLOSE_POS, 30),

		# set aero servo open position: -15deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_AERO, SERVO_SAVE, SERVO_OPEN_POS, -15),

		# set aero servo closed position: +30deg
		minut_servo_info(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_AERO, SERVO_SAVE, SERVO_CLOSE_POS, 30),

		# set flight time-out: 4.5s
		minut_time_out(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, TIME_OUT_SAVE, 45),

		# set flight take-off detection threshold (10 * 10ms, 30 * 0.1G)
		take_off_thres(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, 10, 30),

		# set testing take-off detection threshold (200 * 10ms, 8 * 0.1G)
		#take_off_thres(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, 200, 8),

		# send application start signal
		appli_start(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD),
	],

	#--------------------------------
	# slot #1 : spare
	[
		no_cmde(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD),
	],
]

