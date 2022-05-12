import numpy as np
import matplotlib.pyplot as plt

TEXT_FILE = "log.txt"
VERBOSE_LOCAL_PATH_PLANNER = True

def readLine(line, array_size):
    array = np.zeros((array_size))
    nb_col = 0
    power = 1
    sign = 1
    number = 0
    for c in line:
        if c== "\n":
            array[nb_col] = sign*number
            break
        elif c == ",":
            array[nb_col] = sign*number
            number = 0          
            power = 1
            sign = 1
            nb_col += 1
        elif c == "-":
            sign = -1
        elif c == ".":
            power = -1
        elif c.isnumeric():
            if power == 1:
                number = number*10 + int(c)
            elif power < 0:
                number = number + int(c)*(10**power)
                power -= 1
    return array 

def readLog():
    with open(TEXT_FILE) as file:
        lines = file.readlines()

        log_mode = "nothing"

        set_points = np.empty((0,2))
        poses = np.empty((0,3))

        for i, line in enumerate(lines):
            if VERBOSE_LOCAL_PATH_PLANNER:
                print("Line({}): {}".format(i, line))
            
            if line == "--set point size\n":
                log_mode = "set point size"
                continue
            elif line == "--set points\n":
                log_mode = "set points"
                continue
            elif line == "--poses\n":
                log_mode = "poses"
                continue
            elif log_mode == "set point size":
                set_point_size = readLine(line, array_size=1)[0]
            elif log_mode == "set points":
                array = [readLine(line, array_size=2)]
                set_points = np.append(set_points, array, axis=0)
            elif log_mode == "poses":
                array = [readLine(line, array_size=3)]
                poses = np.append(poses, array, axis=0)

    return set_point_size, set_points, poses

def plotLog(set_point_size, set_points, poses):
    fig = plt.figure()
    ax1 = fig.add_axes([0.1, 0.1, 0.8, 0.8])

    for set_point in set_points:
        circle = plt.Circle((set_point[0], set_point[1]), radius=set_point_size, color='tab:gray', 
                            alpha=0.3, label="SP("+str(set_point[0])+","+str(set_point[1])+")")
        ax1.add_artist(circle)

    color = plt.cm.rainbow(np.linspace(0, 1, poses.shape[0]))
    d_axis = 0.05
    for i, pose in enumerate(poses):
        if not i%10:
            ax1.plot(poses[i,0], poses[i,1], 'o', color=color[i], label="Pose "+str(i))
        else:
            ax1.plot(poses[i,0], poses[i,1], 'o', color=color[i])
        ax1.plot([pose[0], pose[0]+np.cos(pose[2])*d_axis], [pose[1], pose[1]+np.sin(pose[2])*d_axis], color='tab:red', lw=1)
        ax1.plot([pose[0], pose[0]-np.sin(pose[2])*d_axis], [pose[1], pose[1]+np.cos(pose[2])*d_axis], color='tab:green', lw=1)

    ax1.legend()
    plt.xlim(-1.2, 0.7)
    plt.ylim(-0.2, 1.2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title("Local Path Planner")
    plt.xlabel("x axis")
    plt.ylabel("y axis")
    plt.grid()
    fig.savefig('local_path_planner_trajectory.png')

if __name__ == "__main__":
    set_point_size, set_points, poses = readLog()

    if VERBOSE_LOCAL_PATH_PLANNER:
        print(set_point_size)
        print(set_points)
        print(poses)

    plotLog(set_point_size, set_points, poses)
