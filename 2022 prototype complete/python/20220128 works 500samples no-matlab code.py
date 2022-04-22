# Patrick McCorkell
# November 2021
# US Naval Academy
# Robotics and Control TSD
#

print("Hello %s" %__name__)
from time import monotonic_ns, sleep
import board
import busio
import rotaryio
from math import pi
import atexit
from digitalio import DigitalInOut, Direction
from json import loads
from ad5293 import AD5293_309
import ticker



array_size = 1000		# Have gone up to 5000, but at some point this can brick it.
# encoder_pos = []
# encoder_pos.append(0)
# encoder_time = []
# encoder_time.append(monotonic_ns())

def reset():
	import microcontroller
	microcontroller.reset()


def form_arrays():
	global encoder_pos, encoder_time, array_size
	encoder_pos = []
	encoder_time = []
	try:
		for _ in range(array_size):
			encoder_pos.append(0)
			encoder_time.append(0)
		print('arrays formed:')
		print(len(encoder_pos))
	except:
		print("Memory alloc failed.")
		print("Device needs to be restarted.")
		# reset()
form_arrays()

## Things from Matlab
## Placeholders
motor_deadzone = 0
pid = {
	'Kp' : 10000,
	'Ki' : 0,
	'Kd' : 0
}				# Start with just Kp
target_speed = 10		# 10 rad/s

time_limit = 1			# 1 second
controller_update = .002	# 2 ms / 500Hz
print_time = 0.005		# 4ms / 200 Hz


######################################################################
######################################################################
######################################################################

# create function to convert 'rad/sec' to 'count / controller_interval'
# create motor controller class.

######################################################################
######################################################################
######################################################################





###################################
########## Encoder Setup ##########
###################################

enc = rotaryio.IncrementalEncoder(board.GP15,board.GP14,4)
position_last = enc.position
rpm_t_last = monotonic_ns()

encoder_counts_per_rev = 2000



###################################
######## Digital Pot Setup ########
###################################


cs = DigitalInOut(board.GP5)
cs.direction = Direction.OUTPUT
cs.value = 1

sclk = board.GP2
mosi = board.GP3
miso = board.GP4
spi = busio.SPI(sclk,MOSI=mosi,MISO=miso)

digipot = AD5293_309(spi,cs)



###################################
######### Testing Section #########
###################################

# Wrapper for timing function execution.
def tictoc(func):
	def wrapper(*args):
		start = monotonic_ns()
		func(*args)
		end = monotonic_ns()
		# print(func)
		print(str(func)+': '+str((end-start) / (10**9)))
	return wrapper

@tictoc
# from math import floor
def test_func_1():
	for n in range(-65535,65535+1):
		floor(n/65535)

@tictoc
# from ulab import numpy as np
def test_func_2():
	for n in range(-65535,65535+1):
		np.floor(n/65535)


###################################
############# Tickers #############
###################################

interrupts = ticker.Interrupt_Controller()

def encoder_loop():
	global encoder_pos, encoder_time

	encoder_pos.pop(0)
	encoder_time.pop(0)

	encoder_time.append(monotonic_ns())
	encoder_pos.append(enc.position)


angle_factor = (2*pi/encoder_counts_per_rev)
def get_speed():
	global encoder_pos, encoder_time, angle_factor
	pos_diff = (encoder_pos[-1] - encoder_pos[-2])
	time_diff = (encoder_time[-1] - encoder_time[-2])/10**9
	current_speed = (pos_diff / time_diff)
	return current_speed

target_speed = (10 / 6.28) * encoder_counts_per_rev		# 10 rad/s
bias = 70/512
pid = {
	'Kp' : 0.0001,
	'Ki' : 0.00000000001,
	'Kd' : 0.00000000001
}
error = 0
last_error = 0
# @tictoc
def control_loop_P():
	global pid, target_speed, error,bias
	encoder_loop()
	actual = get_speed()
	error = target_speed - actual
	# print(error)
	p_term = pid['Kp'] * error
	# print(actual,error,p_term)
	digipot.set_pot(bias+p_term)

# @tictoc
def control_loop_PI():
	global pid, target_speed, error,bias
	encoder_loop()
	actual = get_speed()
	error = target_speed - actual
	p_term = pid['Kp'] * error
	dt = (encoder_time[-1] - encoder_time[-2])
	i_term = pid['Ki'] * error * dt
	digipot.set_pot(bias+p_term+i_term)

# @tictoc
def control_loop_PID():
	global pid, target_speed, error, last_error, encoder_time,bias
	encoder_loop()
	actual = get_speed()
	error = target_speed - actual
	p_term = pid['Kp'] * error
	dt = (encoder_time[-1] - encoder_time[-2])
	i_term = pid['Ki'] * error * dt
	d_term = pid['Kd'] * (error-last_error) / dt
	digipot.set_pot(bias+p_term+i_term+d_term)
	last_error = error

def trim_arrays():
	index = len(encoder_pos)
	for i in range(1,index+1):
		if (not (encoder_pos[index-i] or encoder_time[index-i])):
			encoder_pos.pop(index-i)
			encoder_time.pop(index-i)
	# print(encoder_time)
	# print(encoder_pos)


q = 0
def check_quit_loop():
	global q, error
	if q:
		digipot.set_pot(0)
		print()
		trim_arrays()
		for i in range(len(encoder_pos)):
			if (encoder_pos[i] or encoder_time[i]):
				print(encoder_pos[i],(encoder_time[i] - encoder_time[0])/10**9)
		print()
		print("Time reached. Pausing program")
		print(error)
		print(len(encoder_pos))
		interrupts.pause()
	else:
		q = 1


speed = 0
step = 1
def control_loop_sawtooth():
	max_speed = 512
	min_speed = -512
	global speed,step
	speed += step
	digipot.set_pot(speed/512)
	if (speed == max_speed):
		step = -1
	elif (speed == min_speed):
		step = 1
	return speed
	# print(speed,get_speed())

def control_loop_find_bias():
	global speed,step, bias
	step = 1
	bias = 0
	if (abs(enc.position) < 10):
		control_loop_sawtooth()
	else:
		digipot.set_pot(0)
		bias = speed/512
		interrupts.pause()


###################################
######## Startup Section ##########
###################################

def runit():
	global encoder_pos, encoder_time, array_size
	if len(encoder_pos) < array_size:
		form_arrays()
	interrupts.interrupt(name='bias',delay=0.1,function=control_loop_find_bias)
	print('finding bias')
	interrupts.loop()
	interrupts.remove_interrupt('bias')
	print('bias found %0.2f' %(bias*512))
	sleep(2)
	# interrupts.interrupt(name='control',delay=controller_update,function=control_loop_sawtooth)
	interrupts.interrupt(name='control',delay=0.0005,function=control_loop_PID)
	interrupts.interrupt(name='quit',delay=time_limit,function=check_quit_loop)
	for _ in range(2):
		encoder_pos.append(enc.position)
		encoder_time.append(monotonic_ns())
	interrupts.loop()


# Under REPL, 'import code' has __name__ attr of 'code'
if __name__ == 'code':
	import sys
	print('REPL detected')

	# Function to reload code.py in REPL after saving changes.
	# User may then 'import code' again to have those changes be live.
	#       - cannot 'import code' from within code.py
	#
	# Usage:
	#       - 'import code'
	#       - make changes in IDE, ctrl+s
	#       - 'code.reload()' to delete the registry
	#       - 'import code' to bring in the new changes
	#
	def reload():
		# exit_program('REPL')
		del sys.modules['code']
	print('reload available')


# This is the default that runs when Ras Pico is plugged in, restarted, ctrl+s, etc.
if __name__ == '__main__':
	runit()



