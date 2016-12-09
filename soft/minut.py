"""
generate the eeprom image for egere minuterie frames
"""


import scalp.scalp
import scalp.scalp_frame

SERVO_CONE = 0xc0
SERVO_AERO = 0xae

SERVO_SAVE = 0x5a
SERVO_READ = 0x4e

SERVO_OPEN_POS = 0x09
SERVO_CLOSE_POS = 0xc1

TIME_OUT_SAVE = 0x00

I2C_SELF_ADDR = scalp.scalp_frame.Scalp.I2C_SELF_ADDR
T_ID = scalp.scalp_frame.Scalp.T_ID
CMD = scalp.scalp_frame.Scalp.CMD


scalps = scalp.scalp.Scalps('./scalp')


# slots number
slots_nb = 2    # up to 10 see scalp_basic.py Container()

slots = [
	#--------------------------------
	# slot #0 : reset
	[
		# set cone servo open position: -15deg
		scalps.ServoInfo(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_CONE, SERVO_SAVE, SERVO_OPEN_POS, -15),

		# set cone servo closed position: +30deg
		scalps.ServoInfo(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_CONE, SERVO_SAVE, SERVO_CLOSE_POS, 30),

		# set aero servo open position: -15deg
		scalps.ServoInfo(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_AERO, SERVO_SAVE, SERVO_OPEN_POS, -15),

		# set aero servo closed position: +30deg
		scalps.ServoInfo(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERVO_AERO, SERVO_SAVE, SERVO_CLOSE_POS, 30),

		# set flight time-out: 4.5s
		scalps.MinutTimeOut(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, TIME_OUT_SAVE, 45),

		# set flight take-off detection threshold (10 * 10ms, 30 * 0.1G)
		scalps.MinutTakeOffThres(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, 10, 30),

		# set testing take-off detection threshold (200 * 10ms, 8 * 0.1G)
		#scalps.take_off_thres(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, 200, 8),
	],

	#--------------------------------
	# slot #1 : spare
	[
		scalps.Null(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD),
	],
]

