/*
	sc18is600.c

	Copyright Yann GOUY <yann_gouy@yahoo.fr>

	partially taken by examples/board_i2c_eeprom
	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>

 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */


//
// quite basic simulation of the SC18IS600 SPI component
//

#include "sc18is600.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sim_avr.h"
#include "avr_spi.h"
#include "avr_twi.h"
#include "avr_ioport.h"		// AVR_IOCTL_IOPORT_GETSTATE()
#include "sim_io.h"			// avr_ioctl()


//--------------------------------------------------------------------
// private defines
//

#define SC18_TX_BUF_SIZE	96
#define SC18_RX_BUF_SIZE	96

#define SC18_SPI_CONF_LSB	0x81
#define SC18_SPI_CONF_MSB	0x42

#define SC18_PWR_STEP_1		0x5a
#define SC18_PWR_STEP_2		0xa5

typedef enum {
	SC18_WR_N = 0x00,
	SC18_RD_N = 0x01,
	SC18_WR_RD = 0x02,
	SC18_RD_BUF = 0x06,
	SC18_WR_WR = 0x03,
	SC18_CONF = 0x18,
	SC18_WR_REG = 0x20,
	SC18_RD_REG = 0x21,
	SC18_PWR = 0x30,
	SC18_NOP = 0xff,    // fake command meaning there no command
} sc18_cmd_t;

typedef enum {
    SC18_FROM_SPI_HOOK,
    SC18_FROM_I2C_HOOK,
} sc18_hook_t;

// internal register map
typedef struct sc18_regs_t {
        uint8_t IOconfig;
        uint8_t IOstate;
        uint8_t I2CClock;
        uint8_t I2CTO;
        uint8_t I2CStat;
        uint8_t I2CAddr;
} _sc18_regs_t;

#define SC18_REGS_OFFSET_MIN	0
#define SC18_REGS_OFFSET_MAX	(sizeof(struct sc18_regs_t) / sizeof(uint8_t))

#define SC18_CS_PB0				0x01
#define SC18_CS_PORT			'B'
#define SC18_CS_PIN				0

typedef enum {
	SC18_SPI_IRQ_IN,
	SC18_SPI_IRQ_OUT,
	SC18_SPI_CS_IRQ,
   	SC18_SPI_IRQ_COUNT,
} _sc18_spi_irq_t;

#define YELLOW_COLOR	"\x1b[93m"
#define PURPLE_COLOR	"\x1b[95m"
#define NORMAL_COLOR	"\x1b[0m"


//--------------------------------------------------------------------
// private constants
//

static const char* const reg_name[] = {
	"IOconfig",
	"IOstate ",
	"I2CClock",
	"I2CTO   ",
	"I2CStat ",
	"I2CAddr ",
};

# define BRIGTH_COLOR   "\x1b[95m"
# define NORMAL_COLOR   "\x1b[0m"

static char* msg2chr[] = {
	BRIGTH_COLOR"0"NORMAL_COLOR,
	BRIGTH_COLOR"["NORMAL_COLOR,
	BRIGTH_COLOR"@"NORMAL_COLOR,
	BRIGTH_COLOR"D"NORMAL_COLOR,
	BRIGTH_COLOR"+"NORMAL_COLOR,
	BRIGTH_COLOR"-"NORMAL_COLOR,
	BRIGTH_COLOR"w"NORMAL_COLOR,
	BRIGTH_COLOR"]"NORMAL_COLOR
};


//--------------------------------------------------------------------
// private structure definitions
//

typedef struct sc18is600_t {
	struct avr_t * avr;		// AVR access for time
	avr_irq_t *	spi_irq;	// irq list for SPI AVR connection
	avr_irq_t *	i2c_irq;	// irq list for I2C bus

    struct sc18_regs_t regs;

	int cs;					// Chip Select

	uint8_t tx_buf[SC18_RX_BUF_SIZE];
	uint8_t rx_buf[SC18_RX_BUF_SIZE];

	sc18_cmd_t cmd;         // current performed command
    int fini;               // flag to signal command performing is ended
    union {
        int step;           // overlapping current step
        struct {
            int step;           // current step
            int index;          // current data index
            int len;            // length of the data to send
            uint8_t i2c_addr;   // target I2C address
        } wr_n;             // write n context
        struct {
            int step;           // current step
            int index;          // current data index
            int len;            // length of the data to send
            uint8_t i2c_addr;   // target I2C address
        } rd_n;             // read n context
    };

	uint8_t reg_offset;         // internal register offset
} sc18is600_t;


//--------------------------------------------------------------------
// private variables
//


//--------------------------------------------------------------------
// private functions
//

// write n bytes to I2C-bus slave device
static void sc18_wr_n(sc18is600_t * sc18, sc18_hook_t hook)
{
	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s src:%s CS:%d st:%d\n", sc18->avr->tag_name, __func__, (hook == SC18_FROM_I2C_HOOK) ? "I2C" : "SPI", sc18->cs, sc18->step);

	avr_twi_msg_irq_t msg;

    if ( hook == SC18_FROM_I2C_HOOK ) {
        // nothing is done when I2C is ACKing
        return;
    }

	// step #0 is setting the command
	if ( sc18->step == 0 ) {
        // send the I2C start
        msg = avr_twi_irq_msg(TWI_MSG_START, 0);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);
        sc18->step++;
        return;
    }

	// step #1 is setting the address
	if ( sc18->step == 1 ) {
        // prepare algo conditions
        sc18->wr_n.index = 3;
        sc18->wr_n.len = sc18->tx_buf[1];
        sc18->wr_n.i2c_addr = sc18->tx_buf[2];

        // send the I2C address
        msg = avr_twi_irq_msg(TWI_MSG_ADDR, sc18->wr_n.i2c_addr);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);
        sc18->step++;
        return;
    }

    // send each byte from tx buffer
    if ( sc18->wr_n.len )  {
        // send the data
        msg = avr_twi_irq_msg(TWI_MSG_DATA, sc18->tx_buf[sc18->wr_n.index]);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);

        // update conditions
        sc18->step++;
        sc18->wr_n.index++;
        sc18->wr_n.len--;
        return;
    }

    // when the data are fully sent
    if ( sc18->wr_n.len == 0 )  {
        // send the I2C stop
        msg = avr_twi_irq_msg(TWI_MSG_STOP, sc18->wr_n.i2c_addr);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);

        // and stop the algo
        sc18->fini = 1;
        return;
    }

    // some boundary checks
    if ( sc18->wr_n.index > SC18_TX_BUF_SIZE ) {
        printf(YELLOW_COLOR"SC18"NORMAL_COLOR": buffer overflow %02d\n", sc18->wr_n.index);

        // force I2C stop
        msg = avr_twi_irq_msg(TWI_MSG_STOP, sc18->wr_n.i2c_addr);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);

        // and algo stop
        sc18->fini = 1;
        return;
    }
}


// read n bytes to I2C-bus slave device
static void sc18_rd_n(sc18is600_t * sc18, sc18_hook_t hook, uint32_t value)
{
	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s src:%s CS:%d st:%d\t", sc18->avr->tag_name, __func__, (hook == SC18_FROM_I2C_HOOK) ? "I2C" : "SPI", sc18->cs, sc18->step);

	avr_twi_msg_irq_t msg = { .v = value };

    if ( hook == SC18_FROM_I2C_HOOK ) {
        // when I2C slave is responding
		printf("msg %s  addr 0x%02x+%c / data 0x%02x\n", msg2chr[msg.bus.msg], msg.bus.data >> 1, msg.bus.data & 0x01 ? 'R' : 'W', msg.bus.data);

        // only store the received data
        sc18->rx_buf[sc18->rd_n.index] = msg.bus.data;
        return;
    }

	printf("\n");

	// step #0: start
	if ( sc18->step == 0 ) {
        // send the I2C start
        msg = avr_twi_irq_msg(TWI_MSG_START, 0);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);
        sc18->step++;
        return;
    }

	// step #1: address
	if ( sc18->step == 1 ) {
        // prepare algo conditions
        sc18->rd_n.index = 0;
        sc18->rd_n.len = sc18->tx_buf[1];
        sc18->rd_n.i2c_addr = sc18->tx_buf[2];

        // send the I2C addr
        msg = avr_twi_irq_msg(TWI_MSG_ADDR, sc18->rd_n.i2c_addr);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);
        sc18->step++;
        return;
    }

    // receive each byte to rx buffer
    if ( sc18->rd_n.len )  {
        // request data from I2C component
        msg = avr_twi_irq_msg(TWI_MSG_CLK, 0);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);

        // update conditions
        sc18->step++;
        sc18->rd_n.index++;
        sc18->rd_n.len--;
        return;
    }

    // when the data are fully received
    if ( sc18->wr_n.len == 0 )  {
        // send the I2C stop
        msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);

        // and stop algo
        sc18->fini = 1;
        return;
    }

    // some boundary checks
    if ( sc18->wr_n.index > SC18_TX_BUF_SIZE ) {
        printf(YELLOW_COLOR"SC18"NORMAL_COLOR": buffer overflow %02d\n", sc18->wr_n.index);

        // force the I2C stop
        msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
        avr_raise_irq(sc18->i2c_irq + SC18_I2C_IRQ_OUT, msg.v);

        // and stop algo
        sc18->fini = 1;
        return;
    }
}


// I2C-bus write then read (read after write)
static void sc18_wr_rd(sc18is600_t * sc18, sc18_hook_t hook)
{
	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s CS:%d st:%d\n", sc18->avr->tag_name, __func__, sc18->cs, sc18->step);

	(void)hook;

    printf(YELLOW_COLOR"SC18"NORMAL_COLOR": not implemented yet! ");
    sc18->fini = 1;

	// step #0 is the command
	if ( sc18->step == 0 ) {
        return;
    }

    // next steps up to limit are for writing the buffer
    if ( sc18->step > SC18_TX_BUF_SIZE ) {
        printf(YELLOW_COLOR"SC18"NORMAL_COLOR": too many data %d ", sc18->step);
        return;
    }
}


// read buffer
static uint8_t sc18_rd_buf(uint8_t value, sc18is600_t * sc18)
{
	printf("%s ", __func__);

	(void)value;

	// step #0 is the command
	if ( sc18->step == 0 ) {
        return 0xff;
    }

    // next steps up to limit are for reading the buffer
    if ( sc18->step > SC18_RX_BUF_SIZE ) {
        printf(" offset out of bound %d ", sc18->step);
        return 0xff;
    }

    // send the content of the response buffer
	return sc18->rx_buf[sc18->step - 1];
}


// I2C-bus write after write
static void sc18_wr_wr(sc18is600_t * sc18, sc18_hook_t hook)
{
	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s CS:%d st:%d\n", sc18->avr->tag_name, __func__, sc18->cs, sc18->step);

	(void)hook;

    printf(YELLOW_COLOR"SC18"NORMAL_COLOR": not implemented yet! ");
    sc18->fini = 1;

	// step #0 is the command
	if ( sc18->step == 0 ) {
        return;
    }

    // next steps up to limit are for sending the buffer
    if ( sc18->step > SC18_TX_BUF_SIZE ) {
        printf(YELLOW_COLOR"SC18"NORMAL_COLOR": write after write: too many data %d\n", sc18->step);
    }
}


// SPI configuration
static uint8_t sc18_conf(uint8_t value, sc18is600_t * sc18)
{
	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s CS:%d st:%d\n", sc18->avr->tag_name, __func__, sc18->cs, sc18->step);

	switch (sc18->step) {
	case 0:
		// command is received
		break;

	case 1:
		// which configuration?
		switch (value) {
		case SC18_SPI_CONF_LSB:
			printf("SC18: conf LSB first ");
			break;

		case SC18_SPI_CONF_MSB:
			printf("SC18: conf MSB first ");
			break;

		default:
			printf("SC18: unknown conf 0x%02x ", value);
			break;
		}

		break;

	default:
		printf("SC18: conf invalid step %02d ", sc18->step);
		break;
	}

	return 0xff;
}


// write internal registers
static uint8_t sc18_wr_reg(uint8_t value, sc18is600_t * sc18)
{
	switch (sc18->step) {
	case 0:
		// command is received
		printf("%s", __func__);
		break;

	case 1:
		// register offset is received
		if ( /*value < SC18_REGS_OFFSET_MIN &&*/ value > SC18_REGS_OFFSET_MAX ) {
			printf("invalid offset %d ", value);
		}
		else {
			sc18->reg_offset = value;
			printf("  %s ", reg_name[value]);
		}
		break;

	case 2:
		printf("    0x%02x   ", value);
		// register value is received
		*((uint8_t*)&sc18->regs + sc18->reg_offset) = value;
		break;

	default:
		printf("SC18: wr_reg invalid step %02d ", sc18->step);
		break;
	}

	return 0xff;
}


// power-down mode
static uint8_t sc18_pwr(uint8_t value, sc18is600_t * sc18)
{
	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s CS:%d st:%d\n", sc18->avr->tag_name, __func__, sc18->cs, sc18->step);

	// sequence 0x30 0x5a 0xa5
	switch (sc18->step) {
	case 0:
		// command
		break;

	case 1:
		if ( value != SC18_PWR_STEP_1 ) {
			printf("invalid power-down step #1 0x%02x ", value);
		}
		break;

	case 2:
		if ( value != SC18_PWR_STEP_2 ) {
			printf("invalid power-down step #2 0x%02x ", value);
		}
		break;

	default:
		printf("power-down invalid sequence %d ", sc18->step);
		break;
	}

	return 0xff;
}


// read internal registers
static uint8_t sc18_rd_reg(uint8_t value, sc18is600_t * sc18)
{
	switch (sc18->step) {
	case 0:
		// command is received
		printf("%s", __func__);
		break;

	case 1:
		// register offset is received
		if ( /*value < SC18_REGS_OFFSET_MIN &&*/ value > SC18_REGS_OFFSET_MAX ) {
			printf(" invalid offset %d", value);
		}
		else {
			sc18->reg_offset = value;
			printf("  %s ", reg_name[value]);
		}
		break;

	case 2:
		value = *((uint8_t*)&sc18->regs + sc18->reg_offset);
		printf("    0x%02x   ", value);
		// register value is sent
		return value;
		break;

	default:
		printf(" invalid step %02d", sc18->step);
		break;
	}

	return 0xff;
}


// called on every SPI transaction
static void sc18_spi_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	(void)irq;

	uint8_t resp;
	sc18is600_t * sc18 = (sc18is600_t*)param;

	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s CS:%d st:%d", sc18->avr->tag_name, __func__, sc18->cs, sc18->step);

	// check if chip is selected
	if ( sc18->cs == 1 ) {
		sc18->step = 0;
		return;
	}

    // new char received
    printf("\t0x%02x --> rx| ", value);

	if ( sc18->step == 0 ) {
		sc18->cmd = value;
	}

	// which command?
	switch (sc18->cmd) {
	case SC18_WR_N:
		printf(" wr n       ");
        sc18->tx_buf[sc18->step] = value & 0xff;
		resp = 0xff;
		break;

	case SC18_RD_N:
		printf(" rd n       ");
        sc18->tx_buf[sc18->step] = value & 0xff;
		resp = 0xff;
		break;

	case SC18_WR_RD:
        sc18->tx_buf[sc18->step] = value & 0xff;
		resp = 0xff;
		break;

	case SC18_WR_WR:
        sc18->tx_buf[sc18->step] = value & 0xff;
		resp = 0xff;
		break;

	case SC18_RD_BUF:
		resp = sc18_rd_buf(value, sc18);
		break;

	case SC18_CONF:
		resp = sc18_conf(value, sc18);
		break;

	case SC18_WR_REG:
		resp = sc18_wr_reg(value, sc18);
		break;

	case SC18_RD_REG:
		resp = sc18_rd_reg(value, sc18);
		break;

	case SC18_PWR:
		resp = sc18_pwr(value, sc18);
		break;

	default:
		resp = 0xff;
		break;
	}

	sc18->step++;

	// send response
    avr_raise_irq(sc18->spi_irq + SC18_SPI_IRQ_OUT, resp);
    printf(" |tx --> 0x%02x\n", resp);
}

// called on every change on CS pin
static void sc18_spi_cs_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	(void)irq;

	sc18is600_t * sc18 = (sc18is600_t*)param;

	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s CS:%d st:%d\n", sc18->avr->tag_name, __func__, sc18->cs, sc18->step);

	sc18->cs = value & SC18_CS_PB0;

	// reset step as component selection changes
    sc18->step = 0;

    // component gets selected
    if ( sc18->cs == 0 ) {
        // reset command as component gets selected
        sc18->cmd = SC18_NOP;
        return;
    }

    // perform the requested I2C transaction
    sc18->fini = 0;
    while ( ! sc18->fini ) {
        switch (sc18->cmd) {
        case SC18_WR_N:
            sc18_wr_n(sc18, SC18_FROM_SPI_HOOK);
            break;

        case SC18_RD_N:
            sc18_rd_n(sc18, SC18_FROM_SPI_HOOK, value);
            break;

        case SC18_WR_RD:
            sc18_wr_rd(sc18, SC18_FROM_SPI_HOOK);
            break;

        case SC18_WR_WR:
            sc18_wr_wr(sc18, SC18_FROM_SPI_HOOK);
            break;

        case SC18_RD_BUF:
        case SC18_CONF:
        case SC18_WR_REG:
        case SC18_RD_REG:
        case SC18_PWR:
        case SC18_NOP:
            sc18->fini = 1;
            break;

        default:
            sc18->fini = 1;
            printf("unknown command 0x%02x\n", sc18->cmd);
            break;
        }
    }
}


// called on every transaction on I2C bus
static void sc18_i2c_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	(void)irq;

	sc18is600_t * sc18 = (sc18is600_t*)param;

	printf(YELLOW_COLOR"SC18"NORMAL_COLOR": ["YELLOW_COLOR"%s"NORMAL_COLOR"] %s CS:%d st:%d\n", sc18->avr->tag_name, __func__, sc18->cs, sc18->step);

    // continue the requested I2C transaction
    switch (sc18->cmd) {
    case SC18_WR_N:
        sc18_wr_n(sc18, SC18_FROM_I2C_HOOK);
        break;

    case SC18_RD_N:
        sc18_rd_n(sc18, SC18_FROM_I2C_HOOK, value);
        break;

    case SC18_WR_RD:
        sc18_wr_rd(sc18, SC18_FROM_I2C_HOOK);
        break;

    case SC18_WR_WR:
        sc18_wr_wr(sc18, SC18_FROM_I2C_HOOK);
        break;

    case SC18_RD_BUF:
    case SC18_CONF:
    case SC18_WR_REG:
    case SC18_RD_REG:
    case SC18_PWR:
    case SC18_NOP:
        break;

    default:
        printf("unknown command 0x%02x ", sc18->cmd);
        break;
    }
}


static const char * spi_irq_names[SC18_SPI_IRQ_COUNT] = {
		[SC18_SPI_IRQ_OUT] = "sc18is600.spi.out",
		[SC18_SPI_IRQ_IN] = "sc18is600.spi.in",
		[SC18_SPI_CS_IRQ] = "sc18is600.spi.cs",
};

static const char * i2c_irq_names[SC18_I2C_IRQ_CNT] = {
		[SC18_I2C_IRQ_OUT] = "sc18is600.i2c.out",
		[SC18_I2C_IRQ_IN] = "sc18is600.i2c.in",
};


// instantiate a SC18IS600 SPI/I2C bridge
struct sc18is600_t* sc18is600_alloc(struct avr_t * avr)
{
	struct sc18is600_t* sc18;

    // allocate and reset the object
    sc18 = malloc(sizeof(sc18is600_t));
	memset(sc18, 0, sizeof(sc18is600_t));

	sc18->avr = avr;

    // init of the SPI side
    sc18->spi_irq = avr_alloc_irq(&avr->irq_pool, SC18_SPI_IRQ_IN, SC18_SPI_IRQ_COUNT, spi_irq_names);

    // connect to the bus
    avr_connect_irq(sc18->spi_irq + SC18_SPI_IRQ_OUT, avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT));
    avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT), sc18->spi_irq + SC18_SPI_IRQ_IN);
	avr_irq_register_notify(sc18->spi_irq + SC18_SPI_IRQ_IN, sc18_spi_hook, sc18);

    // connect the CS
    avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(SC18_CS_PORT), SC18_CS_PIN), sc18->spi_irq + SC18_SPI_CS_IRQ);
	avr_irq_register_notify(sc18->spi_irq + SC18_SPI_CS_IRQ, sc18_spi_cs_hook, sc18);

    // init of the I2C side
    sc18->i2c_irq = avr_alloc_irq(&avr->irq_pool, SC18_I2C_IRQ_IN, SC18_I2C_IRQ_CNT, i2c_irq_names);

    // only register the ISR
    // it is the responsability of the I2C component to connect to this I2C bus
	avr_irq_register_notify(sc18->i2c_irq + SC18_I2C_IRQ_IN, sc18_i2c_hook, sc18);

	return sc18;
}


// release the instantiated SC18IS600 SPI/I2C bridge
void sc18is600_free(struct sc18is600_t* sc18)
{
	free(sc18);
}


// to allow other components to connect to I2C bus
avr_irq_t * sc18is600_i2c_irq_get(struct sc18is600_t* sc18)
{
    return sc18->i2c_irq;
}

