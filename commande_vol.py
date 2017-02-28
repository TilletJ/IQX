#! /usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt
import numpy as np
from time import sleep
from random import random
import heapq
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from quadtree import QuadTree, Node

SEUIL = 80
PROXIMITE = 2
PORTEE = 2

class Tile:
    #a tile of the map and its properties
    def __init__(self, blocked, explored):
        self.blocked = blocked
        self.explored = explored

class Noeud:
    nb_noeud = 0
    def __init__(self, x, y, cout, heuristique):
        self.x = int(x)
        self.y = int(y)
        self.cout = cout
        self.heuristique = heuristique
        self.id = Noeud.nb_noeud
        Noeud.nb_noeud += 1

    def memePoint(self, other):
        return self.x == other.x and self.y == other.y

    def __cmp__(self, other):
        if self.heuristique < other.heuristique:
            return 1
        elif self.heuristique == other.heuristique:
            return 0
        else:
            return -1
    def __hash__(self):
        return self.id

class FilePrio(object):
    """ A neat min-heap wrapper which allows storing items by priority
        and get the lowest item out first (pop()).
        Also implements the iterator-methods, so can be used in a for
        loop, which will loop through all items in increasing priority order.
        Remember that accessing the items like this will iteratively call
        pop(), and hence empties the heap! """

    def __init__(self):
        """ create a new min-heap. """
        self._heap = []

    def push(self, noeud):
        """ Push an item with priority into the heap.
            Priority 0 is the highest, which means that such an item will
            be popped first."""
        heapq.heappush(self._heap, (noeud.heuristique+random()/100, noeud))

    def pop(self):
        """ Returns the item with lowest priority. """
        item = heapq.heappop(self._heap)[1] # (prio, item)[1] == item
        return item

    def liste_valeurs(self):
        result = []
        for e in self._heap:
            result.append(e[1])
        return result

    def __len__(self):
        return len(self._heap)

    def __iter__(self):
        """ Get all elements ordered by asc. priority. """
        return self

    def next(self):
        """ Get all elements ordered by their priority (lowest first). """
        try:
            return self.pop()
        except IndexError:
            raise StopIteration


class Object:
    def __init__(self, x, y, z, blocks=True):
        self.x = int(x)
        self.y = int(y)
        self.z = int(z)
        self.blocks = blocks

    def move(self, dx, dy):
        #move by the given amount, if the destination is not blocked
        if not map[self.x + dx][self.y + dy].blocked:
            self.x += dx
            self.y += dy

    def move2(self, x, y):
        dx = x - self.x
        dy = y - self.y
        self.move(dx, dy)

    def move_towards(self, target_x, target_y):
        """"Envoie les commandes"""
        #vector from this object to the target, and distance

        while dist((target_x, target_y), (int(self.x/0.05), int(self.y/0.05))) > 0.5/0.05:
            dx = target_x - int(self.x/0.05)
            dy = target_y - int(self.y/0.05)
            print "delta " , dx, dy
            print "case actuelle : ", (int(self.x/0.05), int(self.y/0.05))
            print "pos actuelle", self.x, self.y
            sleep(2)


        # distance = sqrt(dx ** 2 + dy ** 2)
        #
        # #normalize it to length 1 (preserving direction), then round it and
        # #convert to integer so the movement is restricted to the map grid
        # dx = round(dx / distance)
        # dy = round(dy / distance)
        # self.move(dx, dy)

    def move_astar(self, target):
        print 'début move astar'
        print target, (int(self.x/0.05), int(self.y/0.05))
        global running_astar
        running_astar = True
        # quadpoints = set()
        # for n in quadmap:
        #     quadpoints.add(n.centre)
        cases_libres, murs_vus = self.lit_quad_frontieres()
        # murs_vus = set(murs_vus).intersection(quadpoints)
        # cases_libres = set(cases_libres).intersection(quadpoints)
        # if target not in cases_libres :
        #     target = plus_proche(target, list(cases_libres))

        print "len cases libres : ", len(cases_libres)

        depart = Noeud(int(self.x/0.05), int(self.y/0.05), 0, 0)

        closedList = []
        openList = FilePrio()
        parent = {}

        openList.push(depart)
        while len(openList) > 0:
            print "len open liste", len(openList)
            u = openList.pop()
            if u.x == target[0] and u.y == target[1]:
                chemin = [u]
                while u in parent:
                    u = parent[u]
                    chemin.append(u)
                for i in range(len(chemin)):
                    chemin[i] = (chemin[i].x, chemin[i].y)
                running_astar = False
                return chemin[::-1][1:]

            for node in quadmap:
                cout = dist(node.centre, (u.x, u.y))

            # for k in range(-1, 2):
            #     for l in range(-1, 2):
            #         # if k==0 or l==0: #pour l'empecher de se déplacer en diago
            #         if k == 0 or l == 0 :
            #             cout = 1 #On se déplace en ligne droite
            #         else:
            #             cout = sqrt(2) #On se déplace en diago (pour éviter que l'algo pense qu'un chemin en zigzag équivaut une ligne droite)
                v = Noeud(node.centre[0], node.centre[1], u.cout + cout, 0) #Si le noeud s'avère intéressant, l'heuristique sera alors calculée
                present = False
                if (v.x, v.y) not in murs_vus:
                    for n in closedList + openList.liste_valeurs():
                        if n.memePoint(v) and n.cout <= v.cout:
                            present = True
                    if not present:
                        v.heuristique = v.cout + dist((v.x, v.y), (target[0], target[1]))
                        openList.push(v)
                        parent[v] = u
            closedList.append(u)
        running_astar = False
        print("Aucun chemin trouvé")
        return []

    def lit_frontieres(self):
        cases_libres = []
        murs_vus = []
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i][j].explored and not map[i][j].blocked:
                    for k in range(-1,2):
                        for l in range(-1,2):
                            if not map[i+k][j+l].explored :
                                    cases_libres.append((i+k,j+l))
                                    break
                elif map[i][j].explored and map[i][j].blocked:
                    murs_vus.append((i,j))
        return cases_libres, murs_vus

    def lit_quad_frontieres(self):
        cases_libres = []
        murs_vus = []
        for node in quadmap:
            if node.occupancy >= 0 and node.occupancy < 100:
                for voisin_pot in quadmap:
                    if dist(voisin_pot.centre, node.centre) <= PORTEE:
                        if voisin_pot.occupancy < 0:
                            cases_libres.append(voisin_pot.centre)
                            break
            elif node.occupancy == 100:
                murs_vus.append(node.centre)
        return cases_libres, murs_vus


    # def move_potentiel(self):
    #     score_max=-9999
    #     objectif = (self.x, self.y)
    #
    #     cases_libres, murs_vus = self.lit_frontieres()
    #
    #     for k in range(-1,2):
    #         for l in range(-1,2):
    #             #if k==0 or l==0:#pour l'empecher de se déplacer en diago
    #             point=(self.x+k, self.y+l)
    #             score=0
    #             for obst in murs_vus:
    #                 d_obst=dist(obst,point)
    #                 if (d_obst**2) == 0 :
    #                     score -=99999
    #                 else:
    #                     score -= 1/(d_obst**2)
    #             for cible in cases_libres:
    #                 d_cible = dist(cible, point)
    #                 score += 50*d_cible**(-2)
    #
    #             if score > score_max:
    #                 score_max = score
    #                 objectif = point
    #
    #     move_x = objectif[0] - self.x
    #     move_y = objectif[1] - self.y
    #     self.move(move_x, move_y)


def dist(x,y):
    return sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2)

def plus_proche(point, liste):
    plus_proche = liste[0]
    dist_min = dist(plus_proche, point)
    for p in liste:
        if dist(point,p) < dist_min:
            plus_proche = p
            dist_min = dist(point,p)
    return plus_proche

def IA_mpv():
    """"Meilleur prochain point de vue."""
    global chemin
    if len(chemin)<1: #On recalcule une nouvelle destination dès qu'on est arrivé pas trop loin de l'objectif (en fonction de la portée du lidar)
        meilleure_pos = (drone.x, drone.y)
        score_max = 0
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i][j].explored and not map[i][j].blocked:
                    compteur = 0
                    for k in range(-1,2):
                        for l in range(-1,2):
                            if not map[i+k][j+l].explored :
                                compteur+=1
                    if compteur > 2:
                        score = compteur**1 / (dist((drone.x, drone.y),(i,j))+0.1)**2
                        if score > score_max:
                            score_max = score
                            meilleure_pos = (i,j)

        #destination = Object(meilleure_pos[0], meilleure_pos[1], 'o', libtcod.white)
        # print(str((drone.x, drone.y)) + " -> " + str(meilleure_pos))
        chemin = drone.move_astar(meilleure_pos)
        print(chemin)

    if len(chemin) == 0: #On a tout exploré !
        print("Exploration terminée")
        return True

    # for point in chemin:
    #     drone.move_towards(point[0], point[1])

    point = chemin.pop(0)
    drone.move_towards(point[0], point[1])

def IA_quad_mpv():
    """"Meilleur prochain point de vue avec le Quadtree."""
    global chemin
    if len(chemin)<1: #On recalcule une nouvelle destination dès qu'on est arrivé pas trop loin de l'objectif (en fonction de la portée du lidar)
        meilleure_pos = (drone.x, drone.y)
        score_max = 0
        for node in quadmap:
            if node.occupancy >= 0 and node.occupancy < 100:
                compteur = 0
                for voisin_pot in quadmap :
                    if dist(voisin_pot.centre, node.centre) <= PORTEE:
                        if voisin_pot.occupancy < 0 :
                            compteur += 1
                if compteur > 2:
                    score = compteur**1 / (dist((int(drone.x/0.05), int(drone.y/0.05)), node.centre)+0.1)**2
                    if score > score_max:
                        score_max = score
                        meilleure_pos = node.centre

        #destination = Object(meilleure_pos[0], meilleure_pos[1], 'o', libtcod.white)
        # print(str((drone.x, drone.y)) + " -> " + str(meilleure_pos))
        chemin = drone.move_astar(meilleure_pos)
        # for i in range(len(chemin)):
        #     chemin[i] = (chemin[i][0] * 0.05, chemin[i][1] * 0.05)
        print(chemin)

    if len(chemin) == 0: #On a tout exploré !
        print("Exploration terminée")
        return True

    # for point in chemin:
    #     drone.move_towards(point[0], point[1])

    point = chemin.pop(0)
    drone.move_towards(point[0], point[1])


def IA_cpp():
    """"Ce qui ne va pas :
    nouvelle destination pas recalculée à chaque tour (possible mais long en calcul)
    quand il y a un mur fin
    """
    global chemin
    if len(chemin)<PROXIMITE: #On recalcule une nouvelle destination dès qu'on est arrivé pas trop loin de l'objectif (en fonction de la portée du lidar)
        meilleure_pos = (drone.x, drone.y)
        min_dist = 99999
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i][j].explored and not map[i][j].blocked:
                    for k in range(-1,2):
                        for l in range(-1,2):
                            if not map[i+k][j+l].explored :
                                if dist((i,j), (drone.x,drone.y)) < min_dist:
                                    meilleure_pos = (i,j)
                                    min_dist = dist((i,j), (drone.x,drone.y))

        #destination = Object(meilleure_pos[0], meilleure_pos[1], 'o', libtcod.white)
        # print(str((drone.x, drone.y)) + " -> " + str(meilleure_pos))
        chemin = drone.move_astar(meilleure_pos)
        # print(chemin)

    if len(chemin) == 0: #On a tout exploré !
        print("Exploration terminée")
        return True

    # for point in chemin:
    #     drone.move_towards(point[0], point[1])

    point = chemin.pop(0)
    drone.move_towards(point[0], point[1])


def map2tile(map, l, h):
    return [[ Tile(map[x,y] > SEUIL, map[x,y] >= 0)
        for y in range(h)]
            for x in range(l)]

def recupere_pos(msg):
    global drone
    drone.x, drone.y, drone.z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    print "source !!!! ", (drone.x, drone.y, drone.z)
    sleep(1)

def param_map(msg):
    print msg.map_load_time
    print msg.resolution
    print msg.width
    print msg.height
    print msg.origin

def affiche_map(msg):
    print msg.info
    print msg.data

def make_map(msg):
    global map, quadmap
    l = msg.info.width
    h = msg.info.height
    #reso = msg.info.resolution
    raw_map = np.array(msg.data).reshape((l,h))
    quadmap = map_light(raw_map, l, h)
    print len(quadmap)
    map = map2tile(raw_map, l ,h)

def map_light(map, l, h):
    Node.map = map
    rootnode = Node(None, (l/2, h/2),(l+h)/2, 100, SEUIL)
    resolution = 4
    tree = QuadTree(rootnode, resolution)
    # for node in QuadTree.leaves:
    #     node.centre = node.centre[0] * 0.05, node.centre[1] * 0.05
    return QuadTree.leaves


if __name__ == "__main__":
    rospy.init_node('recupere_carte')
    drone = Object(0, 0, 0, False)
    chemin = []
    running_astar = False


    while not rospy.is_shutdown():
        sub_pos = rospy.Subscriber('slam_out_pose', PoseStamped, recupere_pos)
        if not running_astar:
            sub_map = rospy.Subscriber('map', OccupancyGrid, make_map)
        sleep(0.01)
        if type(map) is list:
            IA_quad_mpv()