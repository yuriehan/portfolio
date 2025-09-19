"""
Map Search
"""

import comp140_module7 as maps

class Queue:
    """
    A simple implementation of a FIFO queue.
    """
    def __init__(self):
        """ 
        Initialize the queue.
        """
        self._queue = []
    
    def __len__(self):
        """
        Return number of items in the queue.
        """
        return len(self._queue)
    
    def __str__(self):
        """
        Returns a string representation of the queue.
        """
        queue_str = "Queue: " + str(self._queue) 
        return queue_str
    
    def push(self, item):
        """
        Add item to the queue.
        input:
            - item: any data type that's valid in a list
        """  
        return self._queue.append(item)
        
    def pop(self):
        """
        Remove and return the least recently inserted item.
        """
        return self._queue.pop(0)
        
    def clear(self):
        """
        Remove all items from the queue.
        """
        self._queue = []
        return self._queue

class Stack:
    """
    A simple implementation of a LIFO stack.
    """
    def __init__(self):
        """ 
        Initialize the stack.
        """
        self._stack = []
    
    def __len__(self):
        """
        Return number of items in the stack.
        """
        return len(self._stack)
    
    def __str__(self):
        """
        Returns a string representation of the stack.
        """
        stack_str = "Stack: " + str(self._stack) 
        return stack_str
    
    def push(self, item):
        """
        Add item to the stack.
        """  
        return self._stack.append(item)
        
    def pop(self):
        """
        Remove and return the most recently inserted item.
        """
        return self._stack.pop(-1)
        
    def clear(self):
        """
        Remove all items from the stack.
        """
        self._stack = []
        return self._stack



def bfs_dfs(graph, rac_class, start_node, end_node):
    """
    Performs a breadth-first search or a depth-first search on graph
    starting at the start_node.  The rac_class should either be a
    Queue class or a Stack class to select BFS or DFS.

    Completes when end_node is found or entire graph has been
    searched.

    inputs:
        - graph: a directed Graph object representing a street map
        - rac_class: a restricted access container (Queue or Stack) class to
          use for the search
        - start_node: a node in graph representing the start
        - end_node: a node in graph representing the end

    Returns: a dictionary associating each visited node with its parent
    node.
    """

    #same as bfs but replace queue with rac_class
    rac_class = rac_class()
    dist = {}
    parent = {}
    for node in graph.nodes():
        dist[node] = float("inf")
        parent[node] = None
    dist[start_node] = 0
    rac_class.push(start_node)
    #while rac_class is not empty
    while rac_class:
        #remove node from rac_class and attain its neighbors
        node = rac_class.pop()
        nbrs = graph.get_neighbors(node)
        #break out of loop if node reaches goal
        if node == end_node:
            break
        #break out of loop if distance is same as distance of end_node
        if dist[node] == dist[end_node]:
            break
        #iterate through each neighbor of node at consideration
        for nbr in nbrs:
            #if neighbor node has not been looked at yet
            if dist[nbr] == float("inf"):
                dist[nbr] = (dist[node] + 1)
                parent[nbr] = node
                rac_class.push(nbr)
    return parent

#print(bfs_dfs(maps.load_test_graph('line'), Queue, 'A', 'E'))

def dfs(graph, start_node, end_node, parent):
    """
    Performs a recursive depth-first search on graph starting at the
    start_node.

    Completes when end_node is found or entire graph has been
    searched.

    inputs:
        - graph: a directed Graph object representing a street map
        - start_node: a node in graph representing the start
        - end_node: a node in graph representing the end
        - parent: a dictionary that initially has one entry associating
                  the original start_node with None

    Modifies the input parent dictionary to associate each visited node
    with its parent node
    """
    
    #base case
    if start_node == end_node:
        return parent
    #recursive case
    else:
        #get neighbors for node
        nbrs = graph.get_neighbors(start_node)
        #iterate through each neighbor of node
        for nbr in nbrs:
            #if neighbor is not already in parents dictionary
            if nbr not in parent.keys():
                parent[nbr] = start_node
                #run dfs 
                dfs(graph, nbr, end_node, parent)
    return parent
#print(dfs(maps.load_test_graph('line'), 'A', 'E', {'A': None}))
#print(dfs(maps.load_test_graph('clique'), 'A', 'C', {'A': None}))

def astar(graph, start_node, end_node,
          edge_distance, straight_line_distance):
    """
    Performs an A* search on graph starting at start_node.

    Completes when end_node is found or entire graph has been
    searched.

    inputs:
        - graph: a directed Graph object representing a street map
        - start_node: a node in graph representing the start
        - end_node: a node in graph representing the end
        - edge_distance: a function which takes two nodes and a graph
                         and returns the actual distance between two
                         neighboring nodes
        - straight_line_distance: a function which takes two nodes and
                         a graph and returns the straight line distance 
                         between two nodes

    Returns: a dictionary associating each visited node with its parent
    node.
    """
    #initialize all the dictionaries
    g_cost = {}
    h_cost = {}
    f_cost = {}
    parent = {}
    open_set = set()
    closed_set = set()
    
    #first node at consideration is start_node
    open_set.add(start_node)
    parent[start_node] = None
    g_cost[start_node] = 0
    h_cost[start_node] = straight_line_distance(start_node, end_node, graph)
    f_cost[start_node] = g_cost[start_node] + h_cost[start_node]
                                                      
    #while open_set is not empty
    while len(open_set) > 0:
        #get all neighbors for node at consideration
        nbrs = graph.get_neighbors(start_node)
        for nbr in nbrs:
            #h_cost always stays the same for each node
            h_cost[nbr] = straight_line_distance(nbr, end_node, graph)
            #updated g_cost
            g_update = edge_distance(start_node, nbr, graph)
            #if neighbor has not been visited yet, add to open_set
            #and compute g_cost + f_cost
            if nbr not in open_set and nbr not in closed_set:
                open_set.add(nbr)
                parent[nbr] = start_node
                g_cost[nbr] = g_cost[start_node] + g_update
                f_cost[nbr] = g_cost[nbr] + h_cost[nbr]
            #if neighbor has already been visited
            elif nbr in open_set:
                #updating g_cost considering new path used to get to neighbor
                if g_cost[nbr] > (g_cost[start_node] + g_update):
                    g_cost[nbr] = g_cost[start_node] + g_update
                    f_cost[nbr] = g_cost[nbr] + h_cost[nbr]
                    parent[nbr] = start_node
        #move node at consideration from open_set to closed_set
        open_set.remove(start_node)
        closed_set.add(start_node)
        
        #find node in open_set with lowest f_cost
        
        #initialize lowest f_cost as infinity and node_low as nothing
        lowest_f = float('inf')
        node_low = None
        for node in open_set:
            if f_cost[node] < lowest_f:
                #change value of lowest_f and compare rest of nodes' f_costs
                node_low = node
                lowest_f = f_cost[node]
        #make next node at consideration be the node with lowest f_cost
        start_node = node_low
        #if node reaches goal
        if start_node == end_node:
            break
    return parent
    
      
maps.start(bfs_dfs, Queue, Stack, dfs, astar)

