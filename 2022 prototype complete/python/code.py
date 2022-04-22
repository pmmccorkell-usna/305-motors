# Patrick McCorkell
# February 2022
# US Naval Academy
# Robotics and Control TSD
#

from Quanser_305 import Quanser_305
from time import sleep
import atexit
from json import loads
import supervisor
import sys
from tictoc import tictoc
from math import pi
tau = 2*pi

encoder_counts_per_rev = 2000
max_samples = 1000

quanser_305 = Quanser_305(encoder_counts_per_rev, max_samples)



###################################
####### Example Controllers #######
###################################
###################################

error = 0
last_error = 0
i_term = 0
# @tictoc
def control_loop_P():
	global pid, target_speed, error, last_error
	quanser_305.encoder_loop()
	error = target_speed - (quanser_305.get_dx() / quanser_305.get_dt())
	p_term = pid['Kp'] * error
	last_error = error
	# print('p: '+str(p_term)+', i: '+str(0)+', d: '+str(0)+', err: '+str(error))
	quanser_305.digipot.set_pot(bias + p_term)

# @tictoc
def control_loop_PI():
	global pid, target_speed, error, last_error, i_term
	quanser_305.encoder_loop()
	dt = quanser_305.get_dt()
	error = target_speed - (quanser_305.get_dx() / dt)
	p_term = pid['Kp'] * error
	i_term += (pid['Ki'] * error * dt)
	last_error = error
	# print('p: '+str(p_term)+', i: '+str(i_term)+', d: '+str(0)+', err: '+str(error))
	quanser_305.digipot.set_pot(bias + p_term + i_term)

# @tictoc
def control_loop_PID():
	global pid, target_speed, error, last_error, i_term
	quanser_305.encoder_loop()
	dt = quanser_305.get_dt()
	error = target_speed - (quanser_305.get_dx() / dt)
	p_term = pid['Kp'] * error
	i_term += pid['Ki'] * error * dt
	d_term = pid['Kd'] * (error - last_error) / dt
	# print('p: '+str(p_term)+', i: '+str(i_term)+', d: '+str(d_term)+', err: '+str(error))
	quanser_305.digipot.set_pot((bias + p_term + i_term + d_term))
	last_error = error


###################################
##### Control the Controllers #####
###################################
###################################

controller_func_name = control_loop_PID
time_limit = 1
controller_rate = 0.00005
target_speed = (10 / tau) * encoder_counts_per_rev / 10**9		# 10 rad/s
bias = 20/511
sample_offset = 2	# Change how many samples away to get dx and dt.
					# ie,	last sample is 100th pair of position and time.
					# 		If set to 1, compares that to the 99th entry.
					#		To compare against 98th entry, set this to 2.
pid = {
	'Kp' : 0.00003 * 10**9,
	'Ki' : 0.0004,
	'Kd' : 0.00000001  * 10**18
}


def runit():
	# quanser_305.auto_control()
	global controller_rate, time_limit, controller_func_name, target_speed, sample_offset
	quanser_305.change_sample_offset(sample_offset)
	quanser_305.attach_controller(controller_func_name, controller_rate, time_limit)
	quanser_305.run_controller()
	print("LOG: final err: %0.2f%%" %(100 * last_error / target_speed))
	sleep(1)



###################################
######## Matlab Interface #########
############# Section #############
###################################

def format_matlab_values(matlab_data):
	global encoder_counts_per_rev
	global time_limit, controller_rate, target_speed, bias, pid
	conversion_factor = encoder_counts_per_rev / tau

	target_speed = matlab_data['target'] * conversion_factor / 10**9
	time_limit = matlab_data['time_limit']
	controller_rate = max((1 / matlab_data['rate']) - 0.002,0.0005)
	bias = matlab_data['bias'] / 512
	pid['Kp'] = matlab_data['Kp'] * conversion_factor * 10**9
	pid['Ki'] = matlab_data['Ki'] * conversion_factor
	pid['Kd'] = matlab_data['Kd'] * conversion_factor * 10**18
	# print('LOG: ' + str(pid))
	# print('LOG: ' + str(target_speed))

def intake_matlab():
	control_data = {
		'target' : 10,
		'time_limit' : 1,
		'rate' : 500,
		'bias' : 0,
		'Kp' : 9.42e-8, 
		'Ki' : 1.256e-6, 
		'Kd' : 3.14e-11
	}
	# print("LOG: Ready for showtime. Enter parameters.")
	while(1):
		buffer_json = {}
		if supervisor.runtime.serial_bytes_available:
			buffer = sys.stdin.readline()
			try:
				buffer_json = loads(buffer)
				for key in control_data:
					control_data[key] = buffer_json.get(key,control_data[key])
				# print('LOG: intake:'+str(control_data))
			except:
				print('LOG: DataType Error: input was not in correct json format.')
				print("LOG: " + str(buffer))
			format_matlab_values(control_data)
			runit()
			# print("LOG: Lab complete. Enter new parameters in json format.")
			sleep(0.5)



###################################
######## Reset and Program ########
########## Exit Section ###########
###################################


def reset():
	import microcontroller
	microcontroller.reset()

def exit_program():
	quanser_305.digipot.set_pot(0)
	# print('LOG: exiting program')

atexit.register(exit_program)


###################################
######## Startup Section ##########
###################################
###################################

# This is the default that runs when RasPico is plugged in, restarted, ctrl+s, etc.
if __name__ == '__main__':
	# test_pid = {
	# 	'target' : 10,
	# 	'time_limit' : 1,
	# 	'rate' : 500,
	# 	'bias' : 70,
	# 	'Kp' : 0.0000000000000001256,  # 0.00003 * (tau / 2000) / 10**9,
	# 	'Ki' : 0.000001256, # 0.0004 * (tau / 2000),
	# 	'Kd' : 3.14 * 10**-29 # 0.00000001 * (tau/2000) / 10**18
	# }
	# format_matlab_values(test_pid)
	# runit()

	intake_matlab()


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





