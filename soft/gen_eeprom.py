#!/usr/bin/env python3

# format scalps
#
# the scalps are to be stored in EEPROM
#
# the first slot is dedicated to the reset
# the next ones are for the state machine
#
# if there is more than 1 scalp used by the event,
# an container scalp is used and the sequence is coded
# elsewhere in EEPROM.
#

import sys

import scalp
import scalp_frame

import minut


def compute_eeprom(module, fd):
    """compute the content of eeprom memmory for the given module"""
    scls = scalp.Scalps()
    scl = scalp_frame.Scalp()
    scl_size = len(scl)

    offset = module.slots_nb * scl_size    # memory address pointing after the last frame
    mem_map_slot = []            # slots memory map
    mem_map_ext = []            # extended zone memory map (starting at end of slots mem)

    fd.write("//-> %s :\n" % module.__name__)
    #print module.slots
    if len(module.slots) != module.slots_nb:
        raise Exception("slots number inconsistant between declaration and instantiation")

    for s in module.slots:
        #print("%s, len = %d" % (s, len(s)))

        # if current slot contains more than 1 frame
        if len(s) > 1:
            # create a container frame pointing after the last frame in eeprom
            # and copy it in the current slot
            offs_msb = (offset & 0xff00 ) >> 8
            offs_lsb = (offset & 0x00ff ) >> 0
            relay = scls.Container(scl.I2C_SELF_ADDR, scl.I2C_SELF_ADDR, None, scl.CMD, offs_msb, offs_lsb, len(s), scls.Container.EEPROM)
            mem_map_slot.append(relay)
            #print 'offset = 0x%04x' % offset

            # then copy the frames from slot to the end of extended zone
            mem_map_ext.extend(s)
            offset += len(s) * scl_size

        else:
            # else just copy the frame in its slot
            mem_map_slot.extend(s)

        #print("debug slot = %s" % mem_map_slot)
        #print("debug ext = %s" % mem_map_ext)

    mem_map = []
    mem_map.extend(mem_map_slot)
    mem_map.extend(mem_map_ext)

    # fill the C struct
    for i in range(len(mem_map)):
        if i == module.slots_nb:
            fd.write("\n\t//-- start of extended zone --\n")

        fd.write("\t//0x%02x (%3d):" % (i * scl_size, i * scl_size))
        #print(mem_map[i])
        for j in range(scl_size):
            #print(mem_map[i][j]),
            fd.write(" 0x%02x" % mem_map[i][j])

        fd.write(' : %s\n' % mem_map[i])
        fd.write('\t{ ')

        fd.write('.dest = 0x%02x, ' % mem_map[i].dest)
        fd.write('.orig = 0x%02x, ' % mem_map[i].orig)
        fd.write('.t_id = 0x%02x, ' % mem_map[i].t_id)
        fd.write('.cmde = 0x%02x, ' % mem_map[i].cmde)
        fd.write('.status = 0x%02x, ' % mem_map[i].stat)
        fd.write('.argv = {')
        for j in range(scl_size - 5):
            fd.write('0x%02x, ' % mem_map[i][5 + j])
        fd.write('}\n')

        fd.write('\t},\n')


#----------------------------
# main
if __name__ == '__main__':
    fd = open(sys.argv[1], 'w')
    fd.write('#include "dispatcher.h"\n')
    fd.write('\n')
    fd.write('const struct scalp eeprom_scalps[] __attribute__ ((section (".eeprom")))= {\n')

    compute_eeprom(minut, fd)

    fd.write('};')
    fd.close()

