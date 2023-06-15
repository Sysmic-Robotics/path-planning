import random
import math
import pygame

class RRTMap:
    def __init__(self, start, goal,
                 MapDimensions, obsdim, obsnum):
        """Inicio de función

        start: punto de inicio
        goal: punto de llegada
        MapDimensions: dimensión de map
        obsdim: dimensión de los obstáculos
        obsnum: cantidad de obstáculosS
        """
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Map_height, self.Map_width = self.MapDimensions

        # pygame options
        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Map_width, self.Map_height))
        self.map.fill((255, 255, 255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        # obstacles
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        # colors
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    def drawMap(self, obstacles):
        """
        Dibuja los punto inicio y final como círculos
        """
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRad+5, 0)
        pygame.draw.circle(self.map, self.green, self.goal, self.nodeRad+20, 1)
        self.drawObs(obstacles)

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red, node, self.nodeRad+3, 0)

    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)

class RRTGraph:
    def __init__(self, start, goal,
                 MapDimensions, obsdim, obsnum) -> None:
        """Inicio de función

        start: punto de inicio
        goal: punto de llegada
        MapDimensions: dimensión de map
        obsdim: dimensión de los obstáculos
        obsnum: cantidad de obstáculos

        Funciones dentro de la función
        goalFlag: se llega a la región objetivo
        goalstate: indica si el tree llega a región objetivo
        """
        (x,y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.map_height, self.map_width = MapDimensions
        self.x = []
        self.y = []
        self.parent = []

        # inicializar árbol
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # obstáculos
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum

        # path
        self.goalstate = None
        self.path = []

    def makeRandomRect(self):
        """
        Genera un random x, y coordenates for the upper
        left corner of a rectangle. Asegura que no se 
        genere fuera del mapa
        """
        uppercornerx = int(random.uniform(0, self.map_width - self.obsDim))
        uppercornery = int(random.uniform(0, self.map_height - self.obsDim))

        return (uppercornerx, uppercornery)

    # TODO: Los obstáculos son rectángulos
    def makeObs(self):
        """
        Genera obstáculos y los deja en una lista.

        startgoalcol: indica si el start y goal position
        están dentro del obstáculo
        """
        obs = []
        for i in range(0, self.obsNum):
            rectangle = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectangle = pygame.Rect(upper, (self.obsDim, self.obsDim))
                if rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectangle)
        self.obstacles = obs.copy()
        return obs

    def add_node(self, n, x, y):
        """
        Añade un nodo al grafo generado

        n: identificador del nodo
        x, y: coordenadas
        """
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        """
        Retira un nodo al grafo generado

        n: identificador del nodo
        """
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        """
        Añade un arco en la estructura de grafos

        child: índice del nodo hijo
        parent: elemento del nodo padre
        """
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        """
        Corta la conexión entre el nodo con índice n
        y su padre

        n: indice del nodo hijo
        """
        self.parent.pop(n)

    def number_of_nodes(self):
        """
        Retorna la cantidad de nodos totales
        """
        return len(self.x)

    def distance(self, n1, n2):
        """
        Distancia entre dos nodos

        n1: indice de nodo 1
        n2: indice de nodo 2
        """
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2

        return (px + py) ** (0.5)

    def sample_envir(self):
        """
        Se genera un número entre dos valores aleatorios
        definidos por el tamaño del mapa
        """
        x = int(random.uniform(0, self.map_width))
        y = int(random.uniform(0, self.map_height))

        return x, y

    def nearest(self, n):
        """
        Calcula la distancia a todos los nodos del árbol
        e indica el más cercano
        """
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    # TODO: Se identifican como rectángulos
    def isFree(self):
        """
        Indica si algún nodo nuevo está en un lugar libre
        o no.
        """
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()

        while len(obs) > 0:
            rectangle = obs.pop(0)
            if rectangle.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        """
        Encuentra si un vértice que conecta dos nodos
        atraviesa cualquier obstáculo.

        Ocupa interpolación en el arco para comprobar
        colisiones.
        """
        obs = self.obstacles.copy()
        while(len(obs) > 0):
            rectangle = obs.pop(0)
            for i in range(0, 101):
                u = i/100
                x = x1 * u + x2 * (1-u)
                y = y1 * u + y2 * (1-u)
                if rectangle.collidepoint(x,y):
                    return True
        return False

    def connect(self, n1, n2):
        """
        Conectar dos nodos
        """
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax = 35):
        """
        Crea un nodo entre el nodo actual y el más cercano
        con un stepsize
        """
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax/d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand-xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)

            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag
    

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x,y))
        return pathCoords

    def bias(self, ngoal):
        """
        Bias para que vaya en dirección al goal
        """
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def cost(self):
        pass