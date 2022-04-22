# Patrick McCorkell
# January 2022
# US Naval Academy
# Robotics and Control TSD
#
# Built on AD5293 Datasheet Revision E
#




# spi.try_lock()
# spi.configure(phase=1)

# 0x1b 0xff		initialization
# 0x04 0x00		min
# 0x07 0xff		max

from time import monotonic_ns
def tictoc(func):
	def wrapper(*args):
		start = monotonic_ns()
		func(*args)
		end = monotonic_ns()
		# print(func)
		print(str(func)+': '+str((end-start) / (10**9)))
	return wrapper


class AD5293_309():
	def __init__(self,spi_bus,chip_select):
		self._bus = spi_bus
		self._cs = chip_select
		self._cs.value = 1

		# Set range of pot [0,1023]
		self._minn = 0x000
		self._maxn = 0x3ff
		self.lookup_table = {}

		self._ready = False
		attempts = 0
		while ((not self._ready) and (attempts<3)):
			self._init_bus()
			attempts+=1
			print("LOG: Initializing SPI bus. Attempt # "+str(attempts))
		if self._ready:
			print("LOG: SPI bus Initialized.")
			print("LOG: Initializing AD5293 digital potentiometer...")
			self._init_ad5293()
		else:
			print("LOG: AD5293 SPI bus failed to initialize.")
			print("LOG: Timed out. Attempt # "+str(attempts))

	def _init_bus(self):
		self._ready = self._bus.try_lock()
		self._bus.configure(phase=1,polarity=0)	# Data sheet page 4

	def __del__(self):
		self._bus.unlock()

	def _init_ad5293(self):
		self._write([0x1b,0xff])
		self._write([0x06,0x02])

	# def populate_lookup(self):
	# 	resolution = 2000
	# 	for i in range(resolution+1):
	# 		# key = round(i/resolution,4)
	# 		self.lookup_table[i/resolution] = self._transform(i/resolution)

	# write data to the SPI bus
	# data shall be a list of 2 integers, both in range [0,255]
	def _write(self,data):
		# print(data)
		if (len(data) == 2) and (self._ready):
			self._cs.value = 0
			self._bus.write(bytes(data))
			self._cs.value = 1
			return 1
		else:
			print("LOG: Data sent to AD5293._write() of incorrect length.")
			print("LOG: Shall be list[] of length 2. All values in range[0x00,0xFF].")
			return 0

	# Maybe the lookup table is a little too big.
	# This function is sloooooooow.
	# def set_lookup(self,data):
	# 	if len(self.lookup_table) < 1000:
	# 		self.populate_lookup()
	# 	val = self.lookup_table[data]
	# 	update_command = 0x04
	# 	data_LSB = val & 0xff
	# 	data_MSB = ((val & 0xff00) >> 8) | update_command
	# 	self._write([data_MSB,data_LSB])

	# Clamp n to [minn,maxn]
	# @tictoc
	def _clamp(self,n):
		return min(max(n,self._minn),self._maxn)

	# Transform [-1,1] to [0,1023]
	# @tictoc
	def _transform(self,n):
		return self._clamp(int((511.5 * n)+0.5) + 511)	# 1.0166 s / 2048 cycles
		# return self._clamp(round((511.5 * n)+0.5) + 511)	# 1.1025 s / 2048 cycles

		# NO CLAMP:
		# return int((511.5 * n)+0.5) + 511					# 0.9199 s / 2048 cycles
		# return round((511.5 * n)+0.5) + 511				# 0.9971 s / 2048 cycles

	# val shall be [0,1027]
	def set_raw(self,val):
		update_command = 0x04
		data_LSB = val & 0xff
		data_MSB = ((val & 0xff00) >> 8) | update_command
		self._write([data_MSB,data_LSB])

	# val shall be [-1,1]
	def set_pot(self,val):
		# Check that SPI bus is available.
		# if self._ready:

		# See data sheet page 19
		update_command = 0x04
		# print(val)
		val = self._transform(val)
		# print('digipot: '+str(val))

		# Break val into 2 sets of 8bit.
		# See data sheet page 19.
		data_LSB = val & 0xff
		data_MSB = ((val & 0xff00) >> 8) | update_command

		# Send it.
		self._write([data_MSB,data_LSB])

		# 	if (not success):
		# 		print('AD5293.set_pot() failed to write to device.')
		# else:
		# 	print("AD5293.set_pot() failed. SPI bus not available.")







