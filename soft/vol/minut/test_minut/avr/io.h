#ifndef __IO_H__
# define __IO_H__

#include "type_def.h"

extern u8 DDRA;
extern u8 PORTA;

extern u8 DDRB;
extern u8 PORTB;
extern u8 PINB;
#define	PB0	_BV(0)
#define	PB1	_BV(1)
#define	PB2	_BV(2)

#endif	// __IO_H__
