
from RRTBasePy import RRTGraph
import matplotlib.pyplot as plt
import time
import random

def main():
    dimensions = (900, 600)
    start = (200, 200)
    goal = (700, 500)
    obstacles = [([500,400],38)]
    iteration = 0

    graph = RRTGraph(start, goal, dimensions, obstacles)

    while (not graph.path_to_goal()):
        rint = random.randint(0, 100)
        if rint < 80:
            X, Y, Parent = graph.bias(goal)
            pass
        else:
            X, Y, Parent = graph.expand()
            pass
        
        iteration += 1
    #save path in path.txt if not exist create it
    path = graph.getPathCoords()
    f = open("path.txt", "w")
    for i in range(len(path)):
        f.write(str(path[i][0]) + " " + str(path[i][1]) + "\n")
    f.close()
    # plot path and obstacles
    plt.plot([x[0] for x in path], [x[1] for x in path], 'r-')
    for i in range(len(obstacles)):
        circle = plt.Circle((obstacles[i][0][0], obstacles[i][0][1]), obstacles[i][1], color='b')
        plt.gcf().gca().add_artist(circle)
    plt.show()

    return True


        
        # if elapsed > 1:
        #     return True

result = False
t1 = time.time()
while not result:
    elapsed = time.time() - t1
    if elapsed > 1:
        break
    try:
        main()
        result = True
    except:
        pass