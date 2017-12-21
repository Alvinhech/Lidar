import sys
import zmq
import numpy as np

# print sys.version

begin_str = 5
num_str_per_results = 4

def serialization(list_str):
	
	# print list
	id_radar = int(list_str[begin_str])
	time_sec = int(list_str[begin_str+1])
	time_usec = int(list_str[begin_str+2])
	num_results = int(list_str[begin_str+3])

	data_begin_str = begin_str+6
	data = np.array([[0.0 for i in range(num_str_per_results)] for j in range(num_results)])	
	# print '---------------------------------------'
	for i in range(0,num_results):
		for j in range(0,num_str_per_results):
			data[i][j] = float(list_str[data_begin_str+i*num_str_per_results+j])
	
	numsPts_per_class_str = data_begin_str + num_str_per_results*num_results + 2
	numsPts_per_class = np.array([0 for i in range(num_results)])
	for i in range(0,num_results):
		numsPts_per_class[i] = int(list_str[numsPts_per_class_str+i])

	rect_str = numsPts_per_class_str + num_results + 2
	rect_array = np.array([[0 for i in range(4)] for j in range(num_results)])
	for i in range(0,num_results):
		for j in range(0,4):
			rect_array[i][j] = int(list_str[rect_str+i*4+j])

	return id_radar, time_sec, time_usec, num_results, data, numsPts_per_class, rect_array

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.bind("ipc:///tmp/radar_results")

socket.setsockopt(zmq.SUBSCRIBE, 'LADAR')

while True:
	string = socket.recv()
	list_str = string.split();
	# print list_str
	id_radar, time_sec, time_usec, num_results, results, nums_per_class, rect_vec = serialization(list_str)
	if num_results == 0:
		continue;
	print 'id:',id_radar
	print 'time:',time_sec,'s ',time_usec,'us'
	print 'num_result:',num_results
	print results
	print nums_per_class
	print rect_vec
