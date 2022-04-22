from time import monotonic_ns

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




###################################
######### Example Section #########
###################################

# if __name__ == '__main__':
from math import floor
from ulab import numpy as np

@tictoc
def test_func_1():
	for n in range(-65535,65535+1):
		floor(n/65535)

@tictoc
def test_func_2():
	for n in range(-65535,65535+1):
		np.floor(n/65535)
