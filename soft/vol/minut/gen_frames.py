#!/usr/bin/python

# format frames
#
# the frames are to be stored in EEPROM
#
# the 2 first slots are dedicated to the following events :
#	- 0 : reset
#	- 1 : spare
#
# if there is more than 1 frame used by the event,
# an container frame is used and the sequence is coded
# elsewhere in EEPROM.
#

import frame

import minut


def compute_EEPROM(module):
	f = frame.frame()
	fr_size = len(f)

	offset = module.slots_nb * fr_size	# memory address pointing after the last frame
	mem_map_slot = []			# slots memory map
	mem_map_ext = []			# extended zone memory map (starting at end of slots mem)

	print("-> %s : " % module.__name__)
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
			relay = frame.container(frame.I2C_SELF_ADDR, frame.I2C_SELF_ADDR, None, frame.CMD, offs_msb, offs_lsb, len(s), frame.container.EEPROM)
			mem_map_slot.append(relay)
			#print 'offset = 0x%04x' % offset

			# then copy the frames from slot to the end of extended zone
			mem_map_ext.extend(s)
			offset += len(s) * fr_size

		else:
			# else just copy the frame in its slot
			mem_map_slot.extend(s)

		#print("debug slot = %s" % mem_map_slot)
		#print("debug ext = %s" % mem_map_ext)

	mem_map = []
	mem_map.extend(mem_map_slot)
	mem_map.extend(mem_map_ext)

	for i in range(len(mem_map)):
		if i == module.slots_nb:
			print("\n\t-- start of extended zone --")

		print "\t0x%02x (%3d):" % (i * fr_size, i * fr_size),
		#print mem_map[i]
		for j in range(fr_size):
			#print mem_map[i][j],
			print " 0x%02x" % mem_map[i][j],

		print ' : %s' % mem_map[i].cmde_name()

	print

	# create the .hex file to be written in EEPROM
	fd = open(module.__name__ + '_eeprom_frames.hex', 'w')
	fd.write(':02000004008179\n')			# Extended Linear Address Record: offset 0x00810000
	for i in range(len(mem_map)):
		checksum = 0x00

		fd.write(':%02x' % fr_size)			# start code + byte count
		checksum += fr_size

		addr = i * fr_size
		fd.write('%04x' % addr)				# address
		checksum += (addr & 0xff00) >> 8
		checksum += (addr & 0x00ff) >> 0

		fd.write('00')						# record type 00 : data record
		checksum += 0x00

		for j in range(fr_size):			# data
			val = mem_map[i][j]
			fd.write('%02x' % val)
			checksum += val

		checksum &= 0xff					# checksum
		checksum = 0x100 - checksum
		fd.write('%02x' % checksum)
		fd.write('\n')

	fd.close()


#----------------------------
# main
if __name__ == '__main__':
	compute_EEPROM(minut)
