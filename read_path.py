import csv
import matplotlib.pyplot as plt
import sys

def plot_path(file_path, robot_name, relativ):
    x_points = []
    y_points = []
    
    corr = 0.0
    
    if relativ == True:
    	if robot_name == "jetbot_2":
    	    corr = 1.0
    	elif robot_name == "jetbot_3":
    	    corr = -1.0

    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Pomijamy nagłówek
        for row in reader:
            x_points.append(float(row[0]))
            y_points.append(float(row[1])+corr)

    plt.plot(x_points, y_points, linestyle='-', linewidth=2, alpha=0.7, label=robot_name)
    

if __name__ == '__main__':

    robots = ["jetbot_1", "jetbot_2", "jetbot_3"]

    for robot in robots:
        plot_path(f'{robot}_path.csv', robot, True)

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Robots traces")
    plt.legend()
    plt.grid()
    plt.show()
