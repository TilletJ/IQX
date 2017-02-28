import tcod as libtcod
from random import randint, random
from math import sqrt
from time import sleep, time
import numpy as np
import heapq
#actual size of the window
SCREEN_WIDTH = 100
SCREEN_HEIGHT = 60

#size of the map
MAP_WIDTH = 100
MAP_HEIGHT = 55

#parameters for dungeon generator
ROOM_MAX_SIZE = 25
ROOM_MIN_SIZE = 10
MAX_ROOMS = 15


FOV_ALGO = 0  #default FOV algorithm
FOV_LIGHT_WALLS = True  #light walls or not
TORCH_RADIUS = 15

LIMIT_FPS = 20  #20 frames-per-second maximum


color_dark_wall = libtcod.Color(0, 0, 100)
color_light_wall = libtcod.Color(130, 110, 50)
color_dark_ground = libtcod.Color(50, 50, 150)
color_light_ground = libtcod.Color(200, 180, 50)


class Tile:
    #a tile of the map and its properties
    def __init__(self, blocked, block_sight = None):
        self.blocked = blocked

        #all tiles start unexplored
        self.explored = False

        #by default, if a tile is blocked, it also blocks sight
        if block_sight is None: block_sight = blocked
        self.block_sight = block_sight

class Rect:
    #a rectangle on the map. used to characterize a room.
    def __init__(self, x, y, w, h):
        self.x1 = x
        self.y1 = y
        self.x2 = x + w
        self.y2 = y + h
        self.size = w*h

    def center(self):
        center_x = (self.x1 + self.x2) // 2
        center_y = (self.y1 + self.y2) // 2
        return (center_x, center_y)

    def intersect(self, other):
        #returns true if this rectangle intersects with another one
        return (self.x1 <= other.x2 and self.x2 >= other.x1 and
                self.y1 <= other.y2 and self.y2 >= other.y1)

class Noeud:
    def __init__(self, x, y, cout, heuristique):
        self.x = int(x)
        self.y = int(y)
        self.cout = cout
        self.heuristique = heuristique

    def memePoint(self, other):
        return self.x == other.x and self.y == other.y

    def __cmp__(self, other):
        if self.heuristique < other.heuristique:
            return 1
        elif self.heuristique == other.heuristique:
            return 0
        else:
            return -1

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
    #this is a generic object: the player, a monster, an item, the stairs...
    #it's always represented by a character on screen.
    def __init__(self, x, y, char, color, blocks=False):
        self.x = int(x)
        self.y = int(y)
        self.char = char
        self.color = color
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

    def draw(self):
        #only show if it's visible to the player
        if libtcod.map_is_in_fov(fov_map, int(self.x), int(self.y)):
            #set the color and then draw the character that represents this object at its position
            libtcod.console_set_default_foreground(con, self.color)
            libtcod.console_put_char(con, int(self.x), int(self.y), self.char, libtcod.BKGND_NONE)

    def clear(self):
        #erase the character that represents this object
        libtcod.console_put_char(con, int(self.x), int(self.y), ' ', libtcod.BKGND_NONE)

    def move_towards(self, target_x, target_y):
        #vector from this object to the target, and distance
        dx = target_x - self.x
        dy = target_y - self.y
        distance = sqrt(dx ** 2 + dy ** 2)

        #normalize it to length 1 (preserving direction), then round it and
        #convert to integer so the movement is restricted to the map grid
        dx = round(dx / distance)
        dy = round(dy / distance)
        self.move(dx, dy)

    def move_astar(self, target):
        #Create a FOV map that has the dimensions of the map
        fov = libtcod.map_new(MAP_WIDTH, MAP_HEIGHT)

        #Scan the current map each turn and set all the walls as unwalkable
        for y1 in range(MAP_HEIGHT):
            for x1 in range(MAP_WIDTH):
                libtcod.map_set_properties(fov, x1, y1, not map[x1][y1].block_sight, not map[x1][y1].blocked)

        #Scan all the objects to see if there are objects that must be navigated around
        #Check also that the object isn't self or the target (so that the start and the end points are free)
        #The AI class handles the situation if self is next to the target so it will not use this A* function anyway
        for obj in objects:
            if obj.blocks and obj != self and obj != target:
                #Set the tile as a wall so it must be navigated around
                libtcod.map_set_properties(fov, obj.x, obj.y, True, False)

        #Allocate a A* path
        #The 1.41 is the normal diagonal cost of moving, it can be set as 0.0 if diagonal moves are prohibited
        my_path = libtcod.path_new_using_map(fov, 1.41)

        #Compute the path between self's coordinates and the target's coordinates
        libtcod.path_compute(my_path, int(self.x), int(self.y), target.x, target.y)

        #Check if the path exists, and in this case, also the path is shorter than 25 tiles
        #The path size matters if you want the monster to use alternative longer paths (for example through other rooms) if for example the player is in a corridor
        #It makes sense to keep path size relatively low to keep the monsters from running around the map if there's an alternative path really far away
        if not libtcod.path_is_empty(my_path) and libtcod.path_size(my_path) < 50:
            #Find the next coordinates in the computed full path
            x, y = libtcod.path_walk(my_path, True)
            if x or y:
                #Set self's coordinates to the next path tile
                self.x = x
                self.y = y
        else:
            #Keep the old move function as a backup so that if there are no paths (for example another monster blocks a corridor)
            #it will still try to move towards the player (closer to the corridor opening)
            self.move_towards(target.x, target.y)
            return

        #Delete the path to free memory
        libtcod.path_delete(my_path)

    def move_astar2(self, target):
        def plus_proche(decouverts, fScore):
            v_min = np.inf
            p_min = None
            for p in decouverts:
                if fScore[p] < v_min:
                    v_min = fScore[p]
                    p_min = p
            return p_min

        def reconstruct_path(viens_de, current):
            total_path = [current]
            while current in viens_de:
                current = viens_de[current]
                total_path.append(current)
            return total_path

        cases_libres, murs_vus = self.lit_frontieres()
        start = (self.x, self.y)
        proches = set()
        decouverts = {start}
        viens_de = dict()
        gScore, fScore = dict(), dict()
        gScore[start] = 0
        fScore[start] = dist(start, target)

        while len(decouverts) > 0:
            current = plus_proche(decouverts, fScore) #the node in decouverts having the lowest fScore[] value
            if current == target:
                return reconstruct_path(viens_de, current)

            decouverts.remove(current)
            proches.add(current)
            for i in [-1,0,1]:
                for j in [-1,0,1]:
                    if i!=0 or j!=0:
                        voisin = (int(current[0]+i),int(current[0]+j))
                        if voisin in set(cases_libres):
                            if not voisin in proches:
                                tentative_gScore = gScore[current] + 1
                                if voisin not in decouverts: # Discover a new node
                                    decouverts.add(voisin)
                                if tentative_gScore >= gScore[voisin]:
                                    pass # This is not a better path.
                                else:
                                    # This path is the best until now. Record it!
                                    viens_de[voisin] = current
                                    gScore[voisin] = tentative_gScore
                                    fScore[voisin] = gScore[voisin] + dist(voisin, target)

        return -1

    def move_astar3(self, target):
        cases_libres, murs_vus = self.lit_frontieres()
        murs_vus = set(murs_vus)
        depart = Noeud(self.x, self.y, 0, 0)

        closedList = []
        openList = FilePrio()
        parent = dict()

        openList.push(depart)
        while len(openList) > 0:
            u = openList.pop()
            if u.x == target[0] and u.y == target[1]:
                chemin = [u]
                while u in parent:
                    u = parent[u]
                    chemin.append(u)
                for i in range(len(chemin)):
                    chemin[i] = (chemin[i].x, chemin[i].y)
                return chemin[::-1][1:]

            for k in range(-1, 2):
                for l in range(-1, 2):
                    # if k==0 or l==0: #pour l'empecher de se déplacer en diago
                    if k == 0 or l == 0 :
                        cout = 1 #On se déplace en ligne droite
                    else:
                        cout = sqrt(2) #On se déplace en diago (pour éviter que l'algo pense qu'un chemin en zigzag équivaut une ligne droite)
                    v = Noeud(u.x + k, u.y + l, u.cout + cout, 0) #Si le noeud s'avère intéressant, l'heuristique sera alors calculée
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
        print("Aucun chemin trouvé")
        return -1


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


    def move_potentiel(self):
        score_max=-9999
        objectif = (self.x, self.y)

        cases_libres, murs_vus = self.lit_frontieres()

        for k in range(-1,2):
            for l in range(-1,2):
                #if k==0 or l==0:#pour l'empecher de se déplacer en diago
                point=(self.x+k, self.y+l)
                score=0
                for obst in murs_vus:
                    d_obst=dist(obst,point)
                    if (d_obst**2) == 0 :
                        score -=99999
                    else:
                        score -= 1/(d_obst**2)
                for cible in cases_libres:
                    d_cible = dist(cible, point)
                    score += 50*d_cible**(-2)

                if score > score_max:
                    score_max = score
                    objectif = point

        move_x = objectif[0] - self.x
        move_y = objectif[1] - self.y
        self.move(move_x, move_y)


def create_room(room):
    global map
    #go through the tiles in the rectangle and make them passable
    for x in range(room.x1 + 1, room.x2):
        for y in range(room.y1 + 1, room.y2):
            map[x][y].blocked = False
            map[x][y].block_sight = False

    #créer piliers
    for x in range(room.x1 + 1, room.x2):
        for y in range(room.y1 + 1, room.y2):
            if randint(0,1300-room.size)==0:
                map[x][y].blocked = True
                map[x][y].block_sight = True

def create_h_tunnel(x1, x2, y):
    global map
    #horizontal tunnel. min() and max() are used in case x1>x2
    for x in range(int(min(x1, x2)), int(max(x1, x2) + 1)):
        map[x][int(y)].blocked = False
        map[x][int(y)].block_sight = False
        map[x][int(y)+1].blocked = False
        map[x][int(y)+1].block_sight = False
        map[x][int(y)-1].blocked = False
        map[x][int(y)-1].block_sight = False

def create_v_tunnel(y1, y2, x):
    global map
    #vertical tunnel
    for y in range(int(min(y1, y2)), int(max(y1, y2) + 1)):
        map[int(x)][y].blocked = False
        map[int(x)][y].block_sight = False
        map[int(x)+1][y].blocked = False
        map[int(x)+1][y].block_sight = False
        map[int(x)-1][y].blocked = False
        map[int(x)-1][y].block_sight = False


def make_map():
    global map, player

    #fill map with "blocked" tiles
    map = [[ Tile(True)
        for y in range(MAP_HEIGHT) ]
            for x in range(MAP_WIDTH) ]

    rooms = []
    num_rooms = 0

    for r in range(MAX_ROOMS):
        #random width and height
        w = libtcod.random_get_int(0, ROOM_MIN_SIZE, ROOM_MAX_SIZE)
        h = libtcod.random_get_int(0, ROOM_MIN_SIZE, ROOM_MAX_SIZE)
        #random position without going out of the boundaries of the map
        x = libtcod.random_get_int(0, 0, MAP_WIDTH - w - 1)
        y = libtcod.random_get_int(0, 0, MAP_HEIGHT - h - 1)

        #"Rect" class makes rectangles easier to work with
        new_room = Rect(x, y, w, h)

        #run through the other rooms and see if they intersect with this one
        failed = False
        for other_room in rooms:
            if new_room.intersect(other_room):
                failed = True
                break

        if not failed:
            #this means there are no intersections, so this room is valid

            #"paint" it to the map's tiles
            create_room(new_room)

            #center coordinates of new room, will be useful later
            (new_x, new_y) = new_room.center()

            if num_rooms == 0:
                #this is the first room, where the player starts at
                player.x = new_x
                player.y = new_y
            else:
                #all rooms after the first:
                #connect it to the previous room with a tunnel

                #center coordinates of previous room
                (prev_x, prev_y) = rooms[num_rooms-1].center()

                #draw a coin (random number that is either 0 or 1)
                if libtcod.random_get_int(0, 0, 1) == 1:
                    #first move horizontally, then vertically
                    create_h_tunnel(prev_x, new_x, prev_y)
                    create_v_tunnel(prev_y, new_y, new_x)
                else:
                    #first move vertically, then horizontally
                    create_v_tunnel(prev_y, new_y, prev_x)
                    create_h_tunnel(prev_x, new_x, new_y)

            #finally, append the new room to the list
            rooms.append(new_room)
            num_rooms += 1

def dist(x,y):
    return sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2)

def render_all():
    global fov_map, color_dark_wall, color_light_wall
    global color_dark_ground, color_light_ground
    global fov_recompute

    if fov_recompute:
        #recompute FOV if needed (the player moved or something)
        fov_recompute = False
        libtcod.map_compute_fov(fov_map, int(player.x), int(player.y), TORCH_RADIUS, FOV_LIGHT_WALLS, FOV_ALGO)

        #go through all tiles, and set their background color according to the FOV
        for y in range(MAP_HEIGHT):
            for x in range(MAP_WIDTH):
                visible = libtcod.map_is_in_fov(fov_map, x, y)
                wall = map[x][y].block_sight
                if not visible:
                    #if it's not visible right now, the player can only see it if it's explored
                    if map[x][y].explored:
                        if wall:
                            libtcod.console_set_char_background(con, x, y, color_dark_wall, libtcod.BKGND_SET)
                        else:
                            libtcod.console_set_char_background(con, x, y, color_dark_ground, libtcod.BKGND_SET)
                else:
                    #it's visible
                    if wall:
                        libtcod.console_set_char_background(con, x, y, color_light_wall, libtcod.BKGND_SET )
                    else:
                        libtcod.console_set_char_background(con, x, y, color_light_ground, libtcod.BKGND_SET )
                    #since it's visible, explore it
                    map[x][y].explored = True

    #draw all objects in the list
    for object in objects:
        object.draw()

    #blit the contents of "con" to the root console
    libtcod.console_blit(con, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0, 0)

def handle_keys():
    global fov_recompute

    key = libtcod.console_check_for_keypress()  #real-time
    #key = libtcod.console_wait_for_keypress(True)  #turn-based

    if key.vk == libtcod.KEY_ENTER and key.lalt:
        #Alt+Enter: toggle fullscreen
        libtcod.console_set_fullscreen(not libtcod.console_is_fullscreen())

    elif key.vk == libtcod.KEY_ESCAPE:
        return True  #exit game

    #movement keys
    if libtcod.console_is_key_pressed(libtcod.KEY_UP):
        player.move(0, -1)
        fov_recompute = True

    elif libtcod.console_is_key_pressed(libtcod.KEY_DOWN):
        player.move(0, 1)
        fov_recompute = True

    elif libtcod.console_is_key_pressed(libtcod.KEY_LEFT):
        player.move(-1, 0)
        fov_recompute = True

    elif libtcod.console_is_key_pressed(libtcod.KEY_RIGHT):
        player.move(1, 0)
        fov_recompute = True



def IA_mpv():
    """"Meilleur prochain point de vue."""
    global chemin
    if len(chemin)<1: #On recalcule une nouvelle destination dès qu'on est arrivé pas trop loin de l'objectif (en fonction de la portée du lidar)
        meilleure_pos = (player.x, player.y)
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
                        score = compteur**1 / (dist((player.x, player.y),(i,j))+0.1)**2
                        if libtcod.map_is_in_fov(fov_map, int(player.x), int(player.y)):
                            score += 1000
                        if score > score_max:
                            score_max = score
                            meilleure_pos = (i,j)

        #destination = Object(meilleure_pos[0], meilleure_pos[1], 'o', libtcod.white)
        # print(str((player.x, player.y)) + " -> " + str(meilleure_pos))
        chemin = player.move_astar3(meilleure_pos)
        # print(chemin)

    if len(chemin) == 0: #On a tout exploré !
        print("Exploration terminée")
        return True

    # for point in chemin:
    #     player.move_towards(point[0], point[1])

    point = chemin.pop(0)
    player.move_towards(point[0], point[1])

    fov_recompute = True

def IA_cpp():
    """"Ce qui ne va pas :
    nouvelle destination pas recalculée à chaque tour (possible mais long en calcul)
    quand il y a un mur fin
    """
    global chemin
    if len(chemin)<TORCH_RADIUS-1: #On recalcule une nouvelle destination dès qu'on est arrivé pas trop loin de l'objectif (en fonction de la portée du lidar)
        meilleure_pos = (player.x, player.y)
        min_dist = 99999
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i][j].explored and not map[i][j].blocked:
                    for k in range(-1,2):
                        for l in range(-1,2):
                            if not map[i+k][j+l].explored :
                                if dist((i,j), (player.x,player.y)) < min_dist:
                                    meilleure_pos = (i,j)
                                    min_dist = dist((i,j), (player.x,player.y))

        #destination = Object(meilleure_pos[0], meilleure_pos[1], 'o', libtcod.white)
        # print(str((player.x, player.y)) + " -> " + str(meilleure_pos))
        chemin = player.move_astar3(meilleure_pos)
        # print(chemin)

    if len(chemin) == 0: #On a tout exploré !
        print("Exploration terminée")
        return True

    # for point in chemin:
    #     player.move_towards(point[0], point[1])

    point = chemin.pop(0)
    player.move_towards(point[0], point[1])

    fov_recompute = True

def IA_Pledge():
    """Utilise l'algorithme de résolution de labirinthe : Algorithme de Pledge."""
    global angle, compteur #Le robot va dans une direction donnée par cet angle
    pos = (player.x, player.y)
    if compteur == 0: #On va tout droit jusqu'à rencontrer un mur
        if map[int(pos[0]+np.cos(angle))][int(pos[1]+np.sin(angle))].blocked: #S'il y a un mur, on tourne à droite (arbitraire)
            angle -= np.pi/2
            compteur -= 1
    else: #On longe le mur tant que compteur != 0
        if not map[int(pos[0]+np.cos(angle+np.pi/2))][int(pos[1]+np.sin(angle+np.pi/2))].blocked: #Si on peut tourner à gauche
            angle += np.pi / 2
            compteur += 1
        elif map[int(pos[0]+np.cos(angle))][int(pos[1]+np.sin(angle))].blocked: #Si on tombe sur un mur, on tourne à droite
            angle -= np.pi / 2
            compteur -= 1

    player.move(int(np.cos(angle)), int(np.sin(angle)))

    return



#############################################
# Initialization & Main Loop
#############################################

temps_cpp = []

for i in range(12):

    libtcod.console_set_custom_font('arial10x10.png', libtcod.FONT_TYPE_GREYSCALE | libtcod.FONT_LAYOUT_TCOD)
    libtcod.console_init_root(SCREEN_WIDTH, SCREEN_HEIGHT, 'python/libtcod tutorial', False)
    libtcod.sys_set_fps(LIMIT_FPS)
    con = libtcod.console_new(SCREEN_WIDTH, SCREEN_HEIGHT)

    #create object representing the player
    player = Object(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, '@', libtcod.white)

    #create an NPC
    npc = Object(SCREEN_WIDTH/2 - 5, SCREEN_HEIGHT/2, '@', libtcod.yellow)

    #the list of objects with those two
    objects = [npc, player]

    #generate map (at this point it's not drawn to the screen)
    make_map()

    #create the FOV map, according to the generated map
    fov_map = libtcod.map_new(MAP_WIDTH, MAP_HEIGHT)
    for y in range(MAP_HEIGHT):
        for x in range(MAP_WIDTH):
            libtcod.map_set_properties(fov_map, x, y, not map[x][y].block_sight, not map[x][y].blocked)


    fov_recompute = True

    chemin = []
    angle = 0
    compteur = 0
    finish = False
    temps = time()

    #while not libtcod.console_is_window_closed():
    while not finish:

        #render the screen
        render_all()

        IA_cpp()
        #player.move_potentiel()
        #IA_Pledge()
        finish = IA_mpv()
        #sleep(0.01)
        fov_recompute = True

        libtcod.console_flush()

        #erase all objects at their old locations, before they move
        for object in objects:
            object.clear()

        #handle keys and exit game if needed
        exit = handle_keys()
        if exit:
            break
    temps_cpp.append(time()-temps)
    print(time()-temps)
    print(i)

print(temps_cpp)
print(np.mean(temps_cpp))