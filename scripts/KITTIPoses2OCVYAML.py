#############################################################
# Script for converting KITTI ground truth files from KITTI #
# styled to OpenCV styled YAML. To be parsed from OpenCVs   #
# FileStorage module the YAML file needs to fulfill some    #
# specific form conditions.                                 #
#############################################################
import sys

# check for correct input file
if not (len(sys.argv) == 2 and isinstance(sys.argv[1], str)):
    print("Incorrect Number of Input Arguments. Expected at Maximum 1 Argument.\n\
Please pass Name of File you want to convert. Converted File will be stored in same Directory as Original File.")
    exit()

# read yaml file
inputfile = sys.argv[1]
i_file = open(inputfile, 'r')
i_lines = i_file.readlines()
i_file.close()

# write OpenCV specific header to file -> must be there to be parsed from OpenCV
o_lines = "%YAML:1.0\n---\n"

# parse matrices
count = 0
for line in i_lines:
    cols = line.split(' ')
    o_lines += "C" + str(count) + ": !!opencv-matrix\n"
    count += 1
    # set body content to 3x4 matrix
    o_lines += "    rows: 3\n"
    o_lines += "    cols: 4\n"
    # write matrix content
    o_lines += "    dt: d\n    data: [ "
    for i in range(0,len(cols)-1):
            o_lines += cols[i] + ", "
    o_lines += cols[-1].replace("\n", " ]\n")

# write converted data to file
outputfile = inputfile.split('.txt', 1)[0]+"_converted.txt"
o_file = open(outputfile, 'w')
o_file.write(o_lines)
o_file.close()

# print success note
print('File successfully converted!\nFile written to \"{0}\".'.format(outputfile))