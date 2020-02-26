# title : maxflow.py
# class : CSCI3390 Network Science
# authors : Jason Posner, Andres Fernandez, Vignesh Ravi, Nathan Xie

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import ast
import time

# assumptions:
# 1. the input file will contain at least two nodes 1 and 2
# 2. (1) will be the source
# 3. (2) will be the sink
# 4. there is always a path from (1) to (2)


# BFS:

def BFS(G, u):
    n = len(list(G.nodes)) # Get # of vertices
    # range n + 1 because 0 will not be used but len(visited) will be
    Visited = [False for x in range(n+1)] # Set visitied[i] = False for 1 <= i <= n
    S = []
    ToExplore = [] # Queue: ToExplore.
    d = [None for x in range(n+1)] # Gets the predecessor each node (for the tree/path finding)
    # Add u to ToExplore and to S
    ToExplore.append(u)
    S.append(u)
    Visited[int(u)] = True # Visited[u] = True
    # While ToExplore is non-exmpty
    while(len(ToExplore) != 0):
        x = ToExplore.pop(0) # remove x from ToExplore
        Adj_x = list(G.out_edges(x)) # Get Adj(x)
        for (x,y) in Adj_x: # For each edge (x,y) in Adj(x)
            if Visited[int(y)] == False:
                Visited[int(y)] = True
                ToExplore.append(y) # Add y to ToExplore
                S.append(y) # Add y to S
                d[int(y)] = x # Add y to T with edge (x,y)
    return S, d # Output S and the "tree" (predecessor list)

# Helper Functions:

# Gets path to augment flow along
def getPathBFS(G, s, t):
    S, d = BFS(G, s)
    if t not in S: # If t is not reachable from s, no path exists
        return None
    path = [t]
    current = len(d) - 1 # Start from end of predecessor list
    while d[current] != s:
        path.insert(0, d[current]) # Insert it into the path
        current = int(d[current]) # Make its predecessor the current
    path.insert(0, s)
    return path

def getPathCapacity(G, p):
    # For vertices in p
    print("in path capacity")
    print(str(p))
    capacities = [] # List capacities; we want the minimum
    for i in range(len(p) - 1):
        print(str(G[p[i]][p[i+1]]['weight']))
        # fetch first item in path, second -> get weight
        capacities.append(G[p[i]][p[i+1]]['weight'])
    return min(capacities)

def updateResidualGraph(G, p, f):
    for i in range(len(p) - 1):
        u = p[i]
        v = p[i+1]
        G[u][v]['weight'] = G[u][v]['weight'] - f
        if not G.has_edge(v, u):
            G.add_edge(v, u, weight = f)
        else:
            G[v][u]['weight'] = G[u][v]['weight'] + f
        # Check if either edge is zero:
        if G[u][v]['weight'] == 0:
            G.remove_edge(u,v)
        if G[v][u]['weight'] == 0:
            G.remove_edge(v,u)

# Algorithms:

def FordFulkersonBFS(G, s, t):
    G_f = G.copy() # Residual Graph
    f = 0
    path = getPathBFS(G_f, s, t)
    # While there is a flow f' in G_f with v(f') > 0
    while path is not None:
        # If no path remains in the residual graph, we're done
        if path is None:
            break
        f_prime = getPathCapacity(G_f, path) # How much flow we can augment
        f += f_prime # f = f + f'
        updateResidualGraph(G_f, path, f_prime) # Update G_f
        path = getPathBFS(G_f, s, t)
    return f # Output f

def FordFulkersonDijkstra(G, s, t):
    G_f = G.copy() # Residual Graph
    f = 0
    # While there is a flow f' in G_f with v(f') > 0
    while True:
        try:
            path = nx.dijkstra_path(G_f, s, t)
         # If no path remains in the residual graph, we're done
        except nx.NetworkXNoPath:
            break
        f_prime = getPathCapacity(G_f, path) # How much flow we can augment
        f += f_prime # f = f + f'
        updateResidualGraph(G_f, path, f_prime) # Update G_f
    return f # Output f

# takes in the edge list file name and outputs max flow through FF BFS
def maxFlowUsingBFS(fileName):
    # Get G from .txt file:
    G = nx.read_edgelist(fileName, create_using=nx.DiGraph())
    print(FordFulkersonBFS(G,'1', '2'))

# takes in the edge list file name and outputs max flow through FF Dijkstra
def maxFlowUsingDij(fileName):
    # Get G from .txt file:
    G = nx.read_edgelist(fileName, create_using=nx.DiGraph())
    print(FordFulkersonDijkstra(G,'1', '2'))

def compareBFStoDij(fileName):
    # Get G from .txt file:
    G = nx.read_edgelist(fileName, create_using=nx.DiGraph())
    # time both
    start = time.perf_counter()
    count = 1000
    for i in range(count):
        # s = 0 and sink = last numbered node = num of nodes - 1
        FordFulkersonBFS(G,'0', str(G.number_of_nodes() - 1))
    end = time.perf_counter()
    totalTime = (end-start) / count
    result = "Ford Fulkerson BFS: " + str(totalTime)
    print(result)
    # timing of F-F Dijkstra
    start = time.perf_counter()
    for i in range(count):
        # s = 0 and sink = last numbered node = num of nodes - 1
        FordFulkersonDijkstra(G,'0', str(G.number_of_nodes() - 1))
    end = time.perf_counter()
    totalTime2 = (end-start) / count
    result = "Ford Fulkerson Dijkstra: " + str(totalTime2)
    print(result)
    if(totalTime < totalTime2):
        print("BFS was faster")
    else:
        print("Dijkstra was faster")

def plotGraph(fileName):
    G = nx.read_edgelist(fileName, create_using=nx.DiGraph())
    layout = nx.spring_layout(G)
    nx.draw(G, layout, with_labels=True)
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos=layout, edge_labels = labels)
    plt.show()

def main():
    inputFileName = "edge_list2.txt"
    # plotGraph(inputFileName)
    maxFlowUsingBFS(inputFileName)
    # maxFlowUsingDij(inputFileName)
    # compareBFStoDij(inputFileName)


main()
