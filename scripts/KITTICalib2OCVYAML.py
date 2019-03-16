############################################################
# Script for converting KITTI calibration files from KITTI #
# styled to OpenCV styled YAML. To be parsed from OpenCVs  #
# FileStorage module the YAML file needs to fulfill some   #
# specific form conditions.                                #
############################################################
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
o_lines = "%YAML:1.0\n---\n" + i_lines[1]

# parse matrices
for i in range(2,len(i_lines)):
    line = i_lines[i].split(' ')
    o_lines += line[0] + " !!opencv-matrix\n"
    # take first letter of first word per line to switch for body content
    sel = line[0][0]
    if sel == 'S' :   # S_xy 1x2 matrix: image resolution
        o_lines += "    rows: 1\n"
        o_lines += "    cols: 2\n"
    elif sel == 'D' : # D_xy 1x3 matrix: undistortion coefficients
        o_lines += "    rows: 1\n"
        o_lines += "    cols: 5\n"
    elif sel == 'T' : # T_xy 1x3 matrix: translation from camera x to y
        o_lines += "    rows: 1\n"
        o_lines += "    cols: 3\n"
    elif sel == 'P' : # P_xy 3x4 matrix: camera projection
        o_lines += "    rows: 3\n"
        o_lines += "    cols: 4\n"
    else :            # K_xy & R_xy 3x3 matrices: camera intrinsic & rotation from camera x to y
        o_lines += "    rows: 3\n"
        o_lines += "    cols: 3\n"
    # write matrix content
    o_lines += "    dt: d\n    data: [ "
    for i in range(1,len(line)-1):
            o_lines += line[i] + ", "
    o_lines += line[len(line)-1].replace("\n", "") + " ]\n"

# write converted data to file
outputfile = inputfile.split('.txt', 1)[0]+"_converted.txt"
o_file = open(outputfile, 'w')
o_file.write(o_lines)
o_file.close()

# print success note
print('File successfully converted!\nFile written to \"{0}\".'.format(outputfile))