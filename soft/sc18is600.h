//---------------------
//  Copyright (C) 2000-2009  <Yann GOUY>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; see the file COPYING.  If not, write to
//  the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
//  Boston, MA 02111-1307, USA.
//
//  you can write to me at <yann_gouy@yahoo.fr>
//

// the driver is intended for SC18IS600 chip from NXP
//

// usage
//
// sc18is600_init()
//
// send n data to I2C addr
// sc18is600_tx(addr, data, n)
//
// read n data from I2C addr
// sc18is600_rx(addr, data, n)
//

#ifndef __SC18IS600_H__
# define __SC18IS600_H__

# include "type_def.h"

#include "utils/pt.h"


//-----------------------------------------------------
// public types
//


//-----------------------------------------------------
// public functions
//

// initialization of the W5100 component
extern PT_THREAD( SC18IS600_init(pt_t* pt));


// send n data to I2C addr
extern PT_THREAD( SC18IS600_tx(pt_t* pt, u8 addr, u8* data, u8* n));

// read n data from I2C addr
extern PT_THREAD( SC18IS600_rx(pt_t* pt, u8 addr, u8* data, u8* n));

#endif	// __SC18IS600_H__
