# rmse calculation
import numpy as np
import math

# change the path!
inputcfile = "/Users/putiriyadi/Documents local/TU/q4/photogrammetry/A1_Calibration/errorcalc/test_0_12pts.txt"
outputcfile = "/Users/putiriyadi/Documents local/TU/q4/photogrammetry/A1_Calibration/errorcalc/test_0_12pts_output.txt"

# read input coordinate file
f = open(inputcfile,"r")
inputlines = f.readlines()
# horizontal line along x, vertical along y axis
input_coord_h = []
input_coord_v = []
for x in inputlines:
    input_coord_h.append(int(x.split(' ')[3]))
    input_coord_v.append(int(x.split(' ')[4]))
f.close()

# read output coordinate file
of = open(outputcfile,"r")
outputlines = of.readlines()
# horizontal line along x, vertical along y axis
output_coord_h = []
output_coord_v = []
for y in outputlines:
    output_coord_h.append(int(y.split(' ')[3]))
    output_coord_v.append(int(y.split(' ')[4]))
of.close()

h_diff = np.subtract(input_coord_h,output_coord_h)
v_diff = np.subtract(input_coord_v,output_coord_v)
dist_list = []

for i, j in zip(h_diff, v_diff):
    dist = math.sqrt(i*i + j*j)
    dist_list.append(dist)

dist_rmse = math.sqrt(np.square(dist_list).mean())

h_mse = np.square(np.subtract(input_coord_h,output_coord_h)).mean()
h_rmse = math.sqrt(h_mse)
v_mse = np.square(np.subtract(input_coord_v,output_coord_v)).mean()
v_rmse = math.sqrt(v_mse)

print("for ", len(input_coord_h), " correspondent points.")
print("horizontal:")
print(input_coord_h)
print(output_coord_h)
print("horizontal rmse: ", h_rmse)

print("vertical:")
print(input_coord_v)
print(output_coord_v)
print("vertical rmse: ", v_rmse)

print("distance rmse: ", dist_rmse)