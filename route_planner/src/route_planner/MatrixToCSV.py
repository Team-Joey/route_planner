import numpy
import shutil
import os
from os.path import expanduser
from datetime import datetime

# to use function
# import route_planner.MatrixToCSV as writetocsv
# writetocsv.save_to_csv(matrix)

def save_to_csv(matrix):
	a = numpy.asarray(matrix)
	numpy.savetxt("foo.csv", a, delimiter=",")
	home = expanduser("~")
	now = datetime.now()
	current_time = now.strftime("%H_%M_%S")
	os.rename("foo.csv", home + "/catkin_ws/src/route_planner/route_planner/CSV_output/results_"+str(current_time)+".csv")