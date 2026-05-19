class Node:

    # construct this Node with the given id
    def __init__(self, id):
        # used for Dijkstra's implementation
        self.cost = 0.0

        
        self.id = id
        self.outgoing = []
        self.incoming = []

        self.visited = False

        
        self.heap_idx = -1    

        self.pred = None
        
    def __repr__(self):
        return str(self)
        
    # returns a list of links containing the outgoing links of this node

    # returns True if this node is a thru node
    def isThruNode(self):
        return True
  
    # returns the id of this node
    def getId(self):
        return self.id
    
    def __str__(self):
        return str(self.id)

    def addOutgoingLink(self, ij):
        #with open('result39.txt', 'a') as file, contextlib.redirect_stdout(file):
            self.outgoing.append(ij)
            #print(ij)
    
    def addIncomingLink(self, ij):
        #with open('result102.txt', 'a') as file, contextlib.redirect_stdout(file):
            self.incoming.append(ij)
            #print(ij)

    def __lt__(self, other):
        return self.id < other.id