# Patrick McCorkell
# November 2021
# US Naval Academy
# Robotics and Control TSD
#

print("Hello %s" %__name__)
from time import monotonic_ns, sleep
import board
import pulseio
import atexit

###################################
##### Motor and Encoder Setup #####
###################################


###################################
########## Helper Funcs ###########
###################################

# Clamps a value (n) to the range [minn,maxn]
def clamp_val(n, minn, maxn):
	return min(max(n, minn), maxn)


###################################
##### Putting it All Together #####
############ THE LOOP #############
###################################
pwm_in = pulseio.PulseIn(board.GP16,8)
# pwm_in.pause()
# pwm_in.clear()

def read_pulse(): 	# sample_time):
	global pwm_in
	# pwm_in = pulseio.PulseIn(board.GP16,8)
	sample_time = 1

	# Convert sample_time to nanoseconds.
	sample_time *= (10**9)
	q=1

	######## Initial Values ########

	sleep(0.001)    # Just to get a difference between the 2 times, so that dt != 0
	now = monotonic_ns()
	start_time = now


	######## THE LOOP ########

	while(q):
		while ((len(pwm_in)==0) and (q)):
			now = monotonic_ns()
			q = max(start_time + sample_time - now,0)
			print(q)
		print(pwm_in)
		# q=0
		q = max(start_time + sample_time - now,0)
		print(q)
		now = monotonic_ns()



###################################
######### Testing Section #########
###################################

def run_10x_average(func):
	def wrapper():
		vals = []
		for _ in range(10):
			vals.append(func())
		avg = sum(vals)/len(vals)
		# print(str(func)+': '+str(avg))
		return sum(vals)/len(vals)
	return wrapper

def tictoc(func):
	def wrapper():
		start = monotonic_ns()
		func()
		end = monotonic_ns()
		# print(str(func)+': '+str((end-start) / (10**9)))
		return (end-start) / (10**9)
	return wrapper

# run_10x_average (tictoc ( test_func_1))
# g(f(x))
# @run_10x_average
# @tictoc
def test_func_1():
	for n in range(-65535,65535+1):
		floor(n/65535)

# @run_10x_average
# @tictoc
def test_func_2():
	for n in range(-65535,65535+1):
		np.floor(n/65535)

def run_tests():
	global floor, np
	from math import floor
	from ulab import numpy as np
	# print('test_func_1: ' + str(test_func_1()))
	# print('test_func_2: ' + str(test_func_2()))
	decorated_1 = run_10x_average(tictoc(test_func_1))
	decorated_2 = run_10x_average(tictoc(test_func_2))
	print('test_func_1: ' + str(decorated_1()))
	print('test_func_2: ' + str(decorated_2()))


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
		p.duty_cycle=0
	except:
		pass
	try:
		p.deinit()
	except:
		pass

# If code fails or reaches end, execute exit_program() with argument 'atexit'.
atexit.register(exit_program,'atexit')



###################################
######## Startup Section ##########
###################################



import supervisor
# This is the default that runs when Ras Pico is plugged in, restarted, ctrl+s, etc.
if __name__ == '__main__':

	while(1):	
		if supervisor.runtime.serial_bytes_available:
			# print('Serial input detected. Hit enter when finished.')

			sleep(0.1)
	
	exit_program('__main__')


# Under REPL, import code has name 'code'
if __name__ == 'code':
	import sys
	print('REPL detected')

	# Function to reload code.py in REPL after saving changes.
	# User may then 'import code' again to have those changes be live.
	#       - cannot 'import code' from within code.py
	#
	# Usage:
	#       - import code
	#       - make changes in IDE, ctrl+s
	#       - code.reload() to delete the registry
	#       - import code to bring in the new changes
	#
	def reload():
		pwm_in.deinit()
		exit_program('REPL')
		del sys.modules['code']
	print('reload available')
