import argparse
import csv
import re

parser = argparse.ArgumentParser()
parser.add_argument('--static_mode', action='store_true', help="Static path planning instead of dynamic path planning")
parser.add_argument("--csv", type=str, default="massachusetts.csv", help="CSV file with individual metrics for each image")
args = parser.parse_args()

if args.static_mode:
    pathPlannerNames = ['RRTSTAR', 'ASTAR', 'ASTARTHRESHOLD', 'URA']
else:
    pathPlannerNames = ['RRA', 'DSTARLITE', 'URA']

def getMaxValInList(given_list):
    t = 0.0
    for a in range(len(given_list)):
        if float(given_list[a][1]) > t and given_list[a][1] != 'inf':
            t = float(given_list[a][1])
            
    return t

head_data = []
success_rate = [0] * len(pathPlannerNames)
avg_path_acc = [0.0] * len(pathPlannerNames)
norm_path_length = [0.0] * len(pathPlannerNames)
num_images = -1
    
with open(args.csv) as csv_file:

    # creating an object of csv reader
    # with the delimiter as ,
    csv_reader = csv.reader(csv_file, delimiter = ',')

    # list to store the names of columns

    # loop to iterate through the rows of csv
    len_iter = 0
    for row in csv_reader:
        arow = re.split(';+', ''.join(row))[4:]
        if len(row) == 0:
            continue
        elif num_images == -1:
            num_images += 1
            len_iter = len(arow[1:])
            continue
        num_images += 1
        
        # adding the first row
        arow = [s.replace(' ', ',').strip('][').split(',') for s in arow]
        tmp_val = getMaxValInList(arow)
        for a in range(len_iter):
            if arow[a][3] == 'True':
                success_rate[a] += 1
            avg_path_acc[a] += float(arow[a][2])
            if arow[a][1] == 'inf' or (arow[a][1] == '0.0' and arow[a][3] == 'False'):
                norm_path_length[a] += 2 * tmp_val
            else:
                norm_path_length[a] += float(arow[a][1])
                
    print('Algorithm: [Success Rate, Normalized Path Length, Path Accuracy]')
    for z in range(len(pathPlannerNames)):
        success_rate[z] /= num_images
        norm_path_length[z] /= num_images
        avg_path_acc[z] /= num_images
        print(pathPlannerNames[z] + ':', [success_rate[z] * 100, norm_path_length[z], avg_path_acc[z]])

