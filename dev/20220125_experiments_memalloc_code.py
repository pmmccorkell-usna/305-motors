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
from math import pi, floor, ceil
import atexit
from digitalio import DigitalInOut, Direction
from json import loads
from ad5293 import AD5293_309
import ticker



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

# create function to convert 'rad/sec' to 'count / controller_interval'
# create motor controller class that runs the ticker

######################################################################
######################################################################
######################################################################
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

def get_angle():
	return (enc.position % encoder_counts_per_rev) * (360/encoder_counts_per_rev)


def get_rpm():
	global position_last, rpm_t_last
	now = monotonic_ns()
	position_f = enc.position
	diff_position = position_f - position_last
	diff_time = (now - rpm_t_last) / 10**9
	position_last = position_f
	rpm_t_last = now
	return (diff_position / diff_time)


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
########## Helper Funcs ###########
###################################

# Clamps a value (n) to the range [minn,maxn]
def clamp(n, minn, maxn):
	return min(max(n, minn), maxn)


###################################
######## Shutdown Section #########
###################################

# Deinit all the GPIO. Runs automatically at the end.
def exit_program(source):
	print("Quitting program per %s." %source)
	# test_func_1()
	# test_func_2()

	# It's very likely this function can be called more than once depending on the quit conditions and where it was executed from.
	# Therefore, try/except each deinit action. I don't care to see the error messages, I know it was previously deinit'd.
	# try:
	# 	mot_pwm.duty_cycle=0
	# except:
	# 	pass
	# try:
	# 	mot_pwm.deinit()
	# except:
	# 	pass
	# try:
	# 	mot_dir.deinit()
	# except:
	# 	pass
	# try:
	# 	i2c.deinit()
	# except:
	# 	pass
	# try:
	# 	val_in.deinit()
	# except:
	# 	pass
	# try:
	# 	enc.deinit()
	# except:
	# 	pass

# If code fails or reaches end, execute exit_program() with argument 'atexit'.
atexit.register(exit_program,'atexit')

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
def test_func_1():
	for n in range(-65535,65535+1):
		floor(n/65535)

@tictoc
def test_func_2():
	for n in range(-65535,65535+1):
		np.floor(n/65535)

@tictoc
def one_sweep():						# 1.000 s/ 2047 cycles  with clamping and transform [-1,1] to [0,1027]
	for gain in range(-512,512+1):
		digipot.set_pot(gain/512)
	for gain in range(-512,512+1):
		digipot.set_pot((-1*gain)/512)

def sweep():
	while(1):
		for gain in range(-512,512+1):
			digipot.set_pot(gain/512)
		for gain in range(-512,512+1):
			digipot.set_pot((-1*gain)/512)

@tictoc
def one_sweep_manual():					# 0.670898 s / 2048 cycles no error checking
	for gain in range(1024):
		digipot.set_raw(gain)
	for gain in range(1024):
		digipot.set_raw(1023-gain)

@tictoc
def one_sweep_manual_clamp():			# 0.771484 s / 2048 cycles with clamp
	for gain in range(1024):
		digipot.set_raw(clamp(0,1023,gain))
	for gain in range(1024):
		digipot.set_raw(clamp(0,1023,(1023-gain)))

def sweep_manual():
	while(1):
		for gain in range(1024):
			digipot.set_raw(gain)
		for gain in range(1024):
			digipot.set_raw(1023-gain)

@tictoc
def one_sweep_lookup():
	for gain in range(1024):
		gain = round(gain/1024,3)
		digipot.set_lookup(gain)
	for gain in range(1024):
		gain = round(((1023-gain)/1023),3)
		digipot.set_lookup(gain)



###################################
############# Tickers #############
###################################

interrupts = ticker.Interrupt_Controller()

encoder_data =[]
encoder_data.append((0,monotonic_ns()))
def encoder_loop():
	global encoder_data
	if (len(encoder_data)>500):
		encoder_data.pop(0)

	now = monotonic_ns()
	pos = enc.position
	try:
		encoder_data.append((pos,now-encoder_data[0][1]))
	except:
		print('ERROR')
		print(len(encoder_data))
		check_quit_loop()

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
	print(speed,get_speed())


angle_factor = (2*pi/encoder_counts_per_rev)
def get_speed():
	global encoder_data, angle_factor
	pos_diff = (encoder_data[-1][0] - encoder_data[-2][0])
	time_diff = (encoder_data[-1][1] - encoder_data[-2][1])/10**9
	current_speed = (pos_diff / time_diff)
	return current_speed

target_speed = (10 / 6.28) * encoder_counts_per_rev		# 10 rad/s
bias = 70/512
pid = {
	'Kp' : 0.02,
	'Ki' : 0,
	'Kd' : 0
}				# Start with just Kp
error = 0
def control_loop_P():
	global pid, target_speed, error
	encoder_loop()
	actual = get_speed()
	error = target_speed - actual
	# print(error)
	p_term = pid['Kp'] * error
	# print(actual,error,p_term)
	digipot.set_pot(bias+p_term)

def control_loop_find_neg_bias():
	global speed,step
	step = -1
	if (abs(enc.position) < 10):
		control_loop_sawtooth()
	else:
		print(speed)

def control_loop_find_pos_bias():
	global speed,step
	step = 1
	if (abs(enc.position) < 10):
		control_loop_sawtooth()
	else:
		print(speed)

def print_loop():
	global encoder_data,speed
	# diff_pos = encoder_data[-1][0] - encoder_data[-5][0]
	# diff_time = (encoder_data[-1][1] - encoder_data[-5][1]) / 10**9
	# print()
	# print(speed)
	# print(diff_pos/diff_time)
	# print(encoder_data[-1])
	return 1

q = 0
def check_quit_loop():
	global q, error
	if q:
		digipot.set_pot(0)
		print()
		print(encoder_data)
		print("Time reached. Pausing program")
		print(error)
		print(len(encoder_data))
		interrupts.pause()
	else:
		q = 1


###################################
######## Startup Section ##########
###################################


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
		exit_program('REPL')
		del sys.modules['code']
	print('reload available')


# This is the default that runs when Ras Pico is plugged in, restarted, ctrl+s, etc.
if __name__ == '__main__':


	for _ in range(5):
		encoder_loop()
	# interrupts.interrupt(name='encoder',delay=0.002,function=encoder_loop)
	interrupts.interrupt(name='control',delay=0.002,function=control_loop_P)
	# interrupts.interrupt(name='control',delay=controller_update,function=control_loop_find_pos_bias)
	# interrupts.interrupt(name='control',delay=controller_update,function=control_loop_sawtooth)
	# interrupts.interrupt(name='print',delay=print_time,function=print_loop)
	interrupts.interrupt(name='quit',delay=time_limit,function=check_quit_loop)
	interrupts.loop()



	# print((enc.position % encoder_counts_per_rev))
	# # digipot.set_pot(0.2)

	# pos = enc.position
	# last_now = monotonic_ns()
	# start_time = last_now
	# encoder_data = []

	# speed = 0
	# max_speed = 512
	# min_speed = -512
	# step = 1
	# update_speed_delay = 0.01     *10**9
	# digipot.set_pot(0)
	# last_speed_update = last_now

	# update_print_delay = 0.3    *10**9
	# last_print_update = last_now

	# while(1):
	# 	now = monotonic_ns()

	# 	if ((now - last_print_update) > update_print_delay):
	# 		diff_pos = encoder_data[-1][0] - encoder_data[-5][0]
	# 		diff_time = (encoder_data[-1][1] - encoder_data[-5][1]) / 10**9
	# 		last_print_update = now
	# 		print()
	# 		print(speed)
	# 		print(diff_pos/diff_time)

	# 	if ((now - last_speed_update) > update_speed_delay):
	# 		speed += step
	# 		digipot.set_pot(speed/512)
	# 		if (speed == max_speed):
	# 			step = -1
	# 		elif (speed == min_speed):
	# 			step = 1
	# 		last_speed_update = now

	# 	# print(len(encoder_data))
	# 	if (len(encoder_data)>100):
	# 		encoder_data.pop(0)

	# 	last_now = now
	# 	last_position = pos
	# 	now = monotonic_ns()
	# 	pos = enc.position
	# 	encoder_data.append((pos,now))


	# one_sweep()
	# one_sweep_manual()
	# one_sweep_manual_clamp()
	# one_sweep_lookup()

	# sweep_manual()
	# sweep()