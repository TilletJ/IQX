# quadtree.py
# Implements a Node and QuadTree class that can be used as 
# base classes for more sophisticated implementations.
# Malcolm Kesson Dec 19 2012



def get_occ(mini_map, seuil):
    occ = mini_map[0][0]
    if occ < 0:
        for liste in mini_map:
            for value in liste:
                if value >=0 :
                    return 42
        return -1
    elif occ >= 0 and occ < seuil:
        for liste in mini_map:
            for value in liste:
                if value < 0 or value >= seuil :
                    return 42
        return 0
    else:
        for liste in mini_map:
            for value in liste:
                if value < seuil :
                    return 42
        return 100

class Node():
	ROOT = 0
	BRANCH = 1
	LEAF = 2
	minsize = 1   # Set by QuadTree
	map = []
	#_______________________________________________________
	# In the case of a root node "parent" will be None. The
	# "rect" lists the minx,minz,maxx,maxz of the rectangle
	# represented by the node.
	def __init__(self, parent, centre, taille, occupancy, seuil):
		self.parent = parent
		self.children = [None,None,None,None]
		if parent == None:
			self.depth = 0
		else:
			self.depth = parent.depth + 1
		self.occupancy = occupancy
		self.centre = centre
		self.taille = taille
		self.seuil = seuil
		x, y = centre
		if self.parent == None:
			self.type = Node.ROOT
		elif self.occupancy == 0 or taille <= Node.minsize:
			self.type = Node.LEAF
		elif self.occupancy == -1 or taille <= Node.minsize:
			self.type = Node.LEAF
		elif self.occupancy == 100 or taille <= Node.minsize:
			self.type = Node.LEAF
		else: #42
			self.type = Node.BRANCH
	#_______________________________________________________
	# Recursively subdivides a rectangle. Division occurs 
	# ONLY if the rectangle spans a "feature of interest".
	def subdivide(self):
		if self.type == Node.LEAF:
			return
		x,y = self.centre
		h = self.taille/4
		centres = []
		centres.append( (x + h, y + h) )
		centres.append( (x + h, y - h) )
		centres.append( (x - h, y + h) )
		centres.append( (x - h, y - h) )
		for n in range(len(centres)):
			x,y = centres[n]
			occ = get_occ(Node.map[x-h:x+h+1, y-h:y+h+1], self.seuil)
			self.children[n] = Node(self, (x,y), h*2, occ, self.seuil)
			self.children[n].subdivide() # << recursion

#===========================================================			
class QuadTree():
	maxdepth = 1 # the "depth" of the tree
	leaves = []
	allnodes = []
	#_______________________________________________________
	def __init__(self, rootnode, minrect):
		QuadTree.leaves = []
		QuadTree.allnodes = []
		Node.minsize = minrect
		rootnode.subdivide() # constructs the network of nodes
		self.prune(rootnode)
		self.traverse(rootnode)
	#_______________________________________________________
	# Sets children of 'node' to None if they do not have any
	# LEAF nodes.		
	def prune(self, node):
		if node.type == Node.LEAF:
			return 1
		leafcount = 0
		removals = []
		for child in node.children:
			if child != None:
				leafcount += self.prune(child)
				if leafcount == 0:
					removals.append(child)
		for item in removals:
			n = node.children.index(item)
			node.children[n] = None		
		return leafcount
	#_______________________________________________________
	# Appends all nodes to a "generic" list, but only LEAF 
	# nodes are appended to the list of leaves.
	def traverse(self, node):
		QuadTree.allnodes.append(node)
		if node.type == Node.LEAF:
			QuadTree.leaves.append(node)
			if node.depth > QuadTree.maxdepth:
				QuadTree.maxdepth = node.depth
		for child in node.children:
			if child != None:
				self.traverse(child) # << recursion
#_______________________________________________________
# Returns a string containing the rib statement for a
# four sided polygon positioned at height "y".
def RiPolygon(rect, y):	
	x0,z0,x1,z1 = rect
	verts = []
	verts.append(' %1.3f %1.3f %1.3f' % (x0,y,z0))
	verts.append(' %1.3f %1.3f %1.3f' % (x0,y,z1))
	verts.append(' %1.3f %1.3f %1.3f' % (x1,y,z1))
	verts.append(' %1.3f %1.3f %1.3f' % (x1,y,z0))
	rib =  '\tPolygon "P" ['
	rib += ''.join(verts)
	rib += ']\n'
	return rib
		
if __name__=="__main__":
	rootrect = [-2.0, -2.0, 2.0, 2.0]
	resolution = 0.05
	rootnode = Node(None, rootrect)
	tree = QuadTree(rootnode, resolution)
	
	#rect = [-2.0, -2.0, 2.0, 2.0]
	#tree = QuadTree(rect, 0.05)
	print len(QuadTree.leaves)
	print QuadTree.maxdepth
	ribspath = '/Users/mkesson/Documents/WebSite/FUNDZA_COM/algorithmic/quadtree/ribs'
	count = 1
	rib = ''
	for node in QuadTree.leaves:
		fname = 'rect.%0*d.rib' % (4, count)
		path = os.path.join(ribspath, fname)
		f = open(path,'w')
		height = float(node.depth)/QuadTree.maxdepth
		rib += RiPolygon(node.rect, height * 0.25)
		f.write(rib)
		f.close()
		count += 1
	print '---------'








