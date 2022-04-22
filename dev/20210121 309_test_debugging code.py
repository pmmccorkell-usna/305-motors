# Patrick McCorkell
# November 2021
# US Naval Academy
# Robotics and Control TSD
#

print("Hello %s" %__name__)
from time import monotonic_ns, sleep
import board
import rotaryio
from math import pi, floor, ceil
import atexit
import pwmio
from digitalio import DigitalInOut, Direction
from json import loads


###################################
##### Motor and Encoder Setup #####
###################################


mot_pwm = pwmio.PWMOut(pin=board.GP16,frequency=25000,duty_cycle=0)

mot_dir = DigitalInOut(board.GP17)
mot_dir.direction = Direction.OUTPUT
mot_dir.value = 0       # 0 forward, 1 reverse

mot_min_drive = 59000
mot_threshold = 0.5

enc = rotaryio.IncrementalEncoder(board.GP15,board.GP14,4)

encoder_counts_per_rev =  3050 #3140 #3080 #3040 #2500 #1250 #5000

###################################
########## Timing Setup ###########
###################################

interrupt_encoder = 0.01 * (10**9)
interrupt_encoder_sample_time = 0.00005 * (10**9)   
interrupt_motor = 0.0002 * (10**9)


###################################
########## Helper Funcs ###########
###################################

# Clamps a value (n) to the range [minn,maxn]
def clamp_val(n, minn, maxn):
	return min(max(n, minn), maxn)


###################################
########## Motor Funcs ############
###################################

# Calculate rpm.
def calc_rpm(data_0,data_f,counts):
	conversion_factor = 60 * (10**9) # 60 seconds * 10^9 ns/s
	pos_diff = (data_f[0] - data_0[0]) / counts
	time_diff = (data_f[1] - data_0[1])

	# Handle special case when dt = 0.
	# Prevent divide by Zero error.
	if (time_diff):
		return (conversion_factor * pos_diff / time_diff)
	else:
		return 0

# Using a +/- gain value, scale and set the PWM pin and Direction pin accordingly.
def set_pwm(gain):
	abs_gain = abs(gain)
	dir = min(gain/abs_gain,0)
	duty = clamp_val(abs_gain,mot_min_drive,65535)

	mot_dir.value = dir
	mot_pwm.duty_cycle = duty
	# print('set gain %0.2f' %gain)
	# print((dir,duty))
	# print(mot_dir.value)

# Transform [180+ , 360] range to [0- , -180] range.
# No if statements, in loop.
def transform_position(pos):
	low_pos = floor((pos - 180) / 720)
	high_pos = ceil((pos + 180) / 720)
	pos += ((low_pos + high_pos) * -360)
	# print(low_pos)
	# print(high_pos)
	# print('pos: %.2f' %pos)
	return pos

# Get encoder position in terms of degrees [-180,180].
def get_position():
	counts = encoder_counts_per_rev
	return transform_position((enc.position % counts) * (360/counts))

# Transform errors to accomodate fastest route wrt wraparound.
# ie, if your target is > abs(180) degrees away, turn in the opposite direction because it's closer.
# No if statements, in loop.
def get_error(err):
	# print(err)
	low_err = floor((err - 180) / 720)
	high_err = ceil((err + 180) / 720)
	err += ((low_err + high_err) * -360)
	# print(low_err)
	# print(high_err)
	# print(err)
	return err

# Limit the confines to +/- 180 degrees
# Not in a loop, if statements are fine.
def format_target(position):
	if (position > 180):
		position-=360
	if (position < -180):
		position+=360
	return position


###################################
##### Putting it All Together #####
############ THE LOOP #############
###################################

# Variables in:
# - target position in degrees
# - Gains:
#       - either a list of P, PI, or PID.
#           - Must be in that order. If PD, use I of 0. ie (1,0,2) for Kp=1, Ki=0, Kd=2
#           - If trailing values are 0, can be left off. ie (1,2) for Kp=1, Ki=2, Kd=0
#       - or a single value of P
# - Length of time to sample, in seconds. Default of 3s.
def position_control(target_position,gains,sample_time=3.0,min_drive=mot_min_drive):
	global mot_min_drive
	mot_min_drive = min_drive
	# tau = 2*pi
	# conversion_factor = 60 * (10**9) # 60 seconds * 10^9 ns/s

	# Limit the confines to +/- 180 degrees
	target_position = format_target(target_position)

	# Convert sample_time to nanoseconds.
	sample_time *= (10**9)



	######## PID INTAKE ########
	# Convert our times from ns to s; otherwise Ki and Kd become very weird and unintuitive scales.
	conversion_factor = 1 / (10**9)
	
	# setup a zeroed out PID dictionary. This will get updated to intake arguments in a couple lines.
	pid = {
		'Kp' : 0.0,
		'Ki' : 0.0,
		'Kd' : 0.0
	}
	# Check if the input variable is a list.
	if (isinstance(gains,list)):
		# If it is a list, then make sure it's length is not more than 3
		if len(gains)>3:
			print("Error. Incorrect gains.")
			return 0
		# Append 0s until its length is 3.
		for i in range(len(gains),3):
			gains.append(0)
		# Populate the pid dictionary to the gains variable passed in
		for i,gain in enumerate(pid):
			pid[gain] = gains[i]
	# If not a list, then assign Kp to the number.
	else:
		pid['Kp'] = gains


	######## Initial Values ########

	encoder_data = []
	pwm_val,last_position,counter,last_error,i_term=0,0,0,0,0
	threshold = mot_threshold
	q=1
	q_check = 0
	pos = get_position()
	last_now = monotonic_ns()
	sleep(0.001)    # Just to get a difference between the 2 times, so that dt != 0
	now = monotonic_ns()
	start_time = now


	######## THE LOOP ########
	while(q):
		# Calculate Proportional Gain and clamp to +/- 1
		error = get_error(target_position - last_position)
		if abs(error) > threshold:
			q_check = 0
			p_term = pid['Kp'] * error
			i_term += pid['Ki'] * (now-last_now) * conversion_factor * error
			try:
				d_term = pid['Kd'] * ((error-last_error)/((now-last_now) * conversion_factor))
			# print("p: %.2f, i: %.2f, d: %.2f, sum: %.2f" %(p_term,i_term,d_term,p_term+i_term+d_term))
			except ZeroDivisionError:
				d_term = 0

			pwm_val = int(p_term+i_term+d_term)
			set_pwm(pwm_val)

		# If within threshold, stop motor and zero out the Integral
		else:
			# set_pwm(0)
			i_term = 0
			mot_pwm.duty_cycle = 0

			# Quit the loop if position stabilized within the threshold.
			q_check += 1
			if (q_check > 3):
				q = 0

		# Send json data over the line as encoder_data is populated.
		if (len(encoder_data)>1):
			print('{"time": %d, "deg": %.2f, "error": %.2f, "pwm_theoretical": %.2f, "pwm_actual": %.2f, "dir": %d}' %((now-start_time), pos, error, pwm_val, mot_pwm.duty_cycle, mot_dir.value))
			encoder_data.pop(0)

		# Check how much time has elapsed, and set the quit flag if it's more than the alloted sample time.
		q = min(now - start_time - sample_time,0)

		# Update all the variables for the next iteration.
		last_now = now
		last_position = pos
		now = monotonic_ns()
		pos = get_position()
		encoder_data.append((pos,now))
		last_error = error

	# At end of while loop, double check that PWM is zeroed out.
	pwm_val = 0
	mot_pwm.duty_cycle = 0

	# Print the final set of data.
	print('{"time": %d, "deg": %.2f, "error": %.2f, "pwm_theoretical": %.2f, "pwm_actual": %.2f, "dir": %d}' %((now-start_time), pos, error, pwm_val, mot_pwm.duty_cycle, mot_dir.value))
	# print('total samples: %d' %counter)
	# print((now - start_time)/(10**9))
	# print()



###################################
######### Testing Section #########
###################################

# Wrapper for timing function execution.
def tictoc(func):
	def wrapper():
		start = monotonic_ns()
		func()
		end = monotonic_ns()
		print(func.__name__+': '+str((end-start) / (10**9)))
	return wrapper

@tictoc
def test_func_1():
	for n in range(-65535,65535+1):
		floor(n/65535)

@tictoc
def test_func_2():
	for n in range(-65535,65535+1):
		np.floor(n/65535)

def enc_readback():
	while(1):
		print(enc.position)
		print("%.1f" %get_position())
		sleep(0.1)


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
	try:
		mot_pwm.duty_cycle=0
	except:
		pass
	try:
		mot_pwm.deinit()
	except:
		pass
	try:
		mot_dir.deinit()
	except:
		pass
	try:
		i2c.deinit()
	except:
		pass
	try:
		val_in.deinit()
	except:
		pass
	try:
		enc.deinit()
	except:
		pass

# If code fails or reaches end, execute exit_program() with argument 'atexit'.
atexit.register(exit_program,'atexit')



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


# Loop for interfacing JSON with Matlab.
def matlab():
	import supervisor
	import sys

	json_data = {
		'target':0,
		'Kp':4000,
		'Ki':0,
		'Kd':0,
		'runtime':1,	
		'min_drive':mot_min_drive
	}

	while(1):
		# Check if there are bytes being received over serial.
		if supervisor.runtime.serial_bytes_available:
			# print('Serial input detected. Hit enter when finished.')
			data = sys.stdin.readline()
			try:
				# Convert serial JSON into a python dictionary, named 'buffer_json'.
				buffer_json = loads(data)

			except:
				# If conversion failed, zero out the buffer and send an error message.
				buffer_json = 0
				print('DataType Error: input was not in correct json format.')
				print(data)

			# If buffer_json['target'] was properly loaded as json over serial, run position_control.
			if (buffer_json['target']):
				# Populate 'json_data' from the 'buffer_json' dictionary.
				for key in buffer_json:
					json_data[key] = buffer_json[key]
				# print('json: ' + dumps(json_data))
				position_control(json_data['target'],[json_data['Kp'],json_data['Ki'],json_data['Kd']],json_data['runtime'],json_data['min_drive'])

			# Zero out the buffer.
			buffer_json = 0
			sleep(0.1)


# This is the default that runs when Ras Pico is plugged in, restarted, ctrl+s, etc.
if __name__ == '__main__':
	# position_control(-120,[4000,2000,0],2) #with min mot threshold at 9000
	# position_control(desired_angle,4000,3) #with min mot threshold at 10000
	# enc_readback()
	# print("enter data")
	# print(">>")
	matlab()
	exit_program('__main__')
