"""
generate the eeprom image for egere minuterie frames
"""

CULMINATION_TIME = 112 # * 0.1 seconds


import scalp
import scalp_frame

# log
LOG_IN_RAM = 0x14
LOG_FILTER_CMD_ENABLE = 0xc7
LOG_FILTER_RSP_DISABLE = 0x48

# servo
SERV_CONE = 0xc0
SERV_AERO = 0xae

SERV_SAVE = 0x5a
SERV_READ = 0x4e

SERV_OPEN_POS = 0x09
SERV_CLOSE_POS = 0xc1

SERV_OFF = 0x0f
SERV_OPEN = 0x09
SERV_CLOSE = 0xc1

# minut
MINUT_SET = 0x5e

# state
STATE_SET = 0x5e
STATE_INIT = 0x00
STATE_READY = 0x01
STATE_LOCK0 = 0x02
STATE_LOCK1 = 0x03
STATE_LOCK2 = 0x04
STATE_WAITING = 0x05
STATE_THRUSTING = 0x06
STATE_BALISTIC = 0x07
STATE_DETECTION = 0x08
STATE_OPEN_SEQ = 0x09
STATE_BRAKE = 0x0a
STATE_UNLOCK = 0x0b
STATE_PARACHUTE = 0x0c


# led
LED_ALIVE = 0xa1
LED_SIGNAL = 0x51
LED_ERROR = 0xe4

LED_SET = 0x00
LED_GET = 0xff

# misc
I2C_SELF_ADDR = scalp_frame.Scalp.I2C_SELF_ADDR
I2C_BRD_ADDR = scalp_frame.Scalp.I2C_BROADCAST_ADDR
T_ID = scalp_frame.Scalp.T_ID
CMD = scalp_frame.Scalp.CMD


scalps = scalp.Scalps('./scalp')


slots = [
    #--------------------------------
    # slot #0 : reset
    [
        # enable all scalps to be logged
        #scalps.Log(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LOG_FILTER_ENABLE, 0xff, 0xff, 0xff, 0xff),
        # enable all scalps to be logged except TWIREAD (0x01) and TWIWRITE (0x02)
        #scalps.Log(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LOG_FILTER_ENABLE, 0xff, 0xff, 0xff, 0xf9),
        # enable only SCALP_STATE (0x06) and SCALP_MINUTEVENT (0x16) scalps to be logged
        scalps.Log(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LOG_FILTER_CMD_ENABLE, 0x00, 0x40, 0x00, 0x40),
        # disable all response scalps to be logged
        scalps.Log(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LOG_FILTER_RSP_DISABLE, 0xff, 0xff, 0xff, 0xff),

        # enable log in RAM
        scalps.Log(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LOG_IN_RAM),

        # set cone servo open position: -15deg
        scalps.ServoInfo(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_SAVE, SERV_OPEN_POS, -15),

        # set cone servo closed position: +30deg
        scalps.ServoInfo(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_SAVE, SERV_CLOSE_POS, 30),

        # set aero servo open position: -15deg
        scalps.ServoInfo(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_SAVE, SERV_OPEN_POS, -15),

        # set aero servo closed position: +30deg
        scalps.ServoInfo(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_SAVE, SERV_CLOSE_POS, 30),

        # set flight thresholds (40 * 0.1G, 5 * 0.1G, 80 * 0.01GG)
        scalps.MinutTakeOffThres(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, 40, 5, 80),

        # led alive 0.1/0.1s, signal 0.4/0.1s, error 0.0/12.7s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ALIVE, LED_SET, 1, 1),
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_SIGNAL, LED_SET, 4, 1),
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ERROR, LED_SET, 0, 127),
    ],

    #--------------------------------
    # slot #1 : minut action init
    [
        # signal init state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_INIT),

        # cmde cone stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_OFF),

        # cmde aero stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_OFF),

        # led alive 2.0/0.5s, signal 0.1/0.1s, error 12.7/0.0s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ALIVE, LED_SET, 20, 5),
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_SIGNAL, LED_SET, 1, 1),
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ERROR, LED_SET, 127, 0),
    ],

    #--------------------------------
    # slot #2 : minut action ready
    [
        # signal ready state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_READY),

        # led error 12.7/0.0s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ERROR, LED_SET, 127, 0),
    ],

    #--------------------------------
    # slot #3 : minut action lock0
    [
        # signal lock0 state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_LOCK0),

        # cmde cone open
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_OPEN),

        # cmde aero open
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_OPEN),

        # led signal 0.0/1.0s, error 0.1/0.1s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_SIGNAL, LED_SET, 0, 10),
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ERROR, LED_SET, 1, 1),
    ],

    #--------------------------------
    # slot #4 : minut action lock1
    [
        # signal lock1 state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_LOCK1),

        # cmde aero close
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_CLOSE),

        # led signal 1.0/0.0s, error 0.0/1.0s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_SIGNAL, LED_SET, 10, 0),
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ERROR, LED_SET, 0, 10),

        # minut time-out set to +5 s
        scalps.MinutTimeOut(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, MINUT_SET, 50),
    ],

    #--------------------------------
    # slot #5 : minut action lock2
    [
        # signal lock2 state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_LOCK2),

        # cmde cone close
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_CLOSE),

        # led error 1.0/0.0s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ERROR, LED_SET, 10, 0),

        # minut time-out set to +1 s
        scalps.MinutTimeOut(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, MINUT_SET, 10),
    ],

    #--------------------------------
    # slot #6 : minut action waiting
    [
        # signal waiting state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_WAITING),

        # cmde cone stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_OFF),

        # cmde aero stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_OFF),

        # led signal 2.0/0.4s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_SIGNAL, LED_SET, 20, 4),
    ],

    #--------------------------------
    # slot #7 : minut action thrusting
    [
        # signal flight state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_THRUSTING),

        # led error 0.1/0.1s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ERROR, LED_SET, 1, 1),

        # minut time-out set to culmination time - 1.0 s (to set enable the detection)
        scalps.MinutTimeOut(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, MINUT_SET, CULMINATION_TIME - 10),
    ],

    #--------------------------------
    # slot #8 : minut action balistic
    [
        # signal balistic state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_BALISTIC),
    ],

    #--------------------------------
    # slot #9 : minut action detection
    [
        # signal detection state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_DETECTION),

        # minut time-out set to +2.0 s
        scalps.MinutTimeOut(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, MINUT_SET, 20),
    ],

    #--------------------------------
    # slot #a : minut action open_seq
    [
        # signal open seq state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_OPEN_SEQ),

        # cmde cone open
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_OPEN),

        # cmde aero stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_OFF),

        # led signal 0.2/0.2s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_SIGNAL, LED_SET, 2, 2),

        # minut time-out set to +0.2 s
        scalps.MinutTimeOut(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, MINUT_SET, 2),
    ],

    #--------------------------------
    # slot #b : minut action brake
    [
        # signal brake state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_BRAKE),

        # cmde cone stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_OFF),

        # cmde aero open
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_OPEN),

        # minut time-out set to +0.2 s
        scalps.MinutTimeOut(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, MINUT_SET, 2),
    ],

    #--------------------------------
    # slot #c : minut action unlock
    [
        # signal unlock state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_UNLOCK),

        # cmde cone open
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_OPEN),

        # cmde aero stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_OFF),

        # minut time-out set to +0.2 s
        scalps.MinutTimeOut(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, MINUT_SET, 2),
    ],

    #--------------------------------
    # slot #d : minut action parachute
    [
        # signal parachute state
        scalps.State(I2C_BRD_ADDR, I2C_SELF_ADDR, T_ID, CMD, STATE_SET, STATE_PARACHUTE),

        # cmde cone stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_CONE, SERV_OFF),

        # cmde aero stop
        scalps.ServoCmd(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, SERV_AERO, SERV_OFF),

        # led signal 2.5/1.0s, error 2.5/1.0s
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_SIGNAL, LED_SET, 25, 10),
        scalps.Led(I2C_SELF_ADDR, I2C_SELF_ADDR, T_ID, CMD, LED_ERROR, LED_SET, 25, 10),
    ],
]

# slots number
slots_nb = len(slots)    # up to 10 see scalp_basic.py Container()
