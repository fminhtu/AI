# Contents:
# 1. DFS & BFS
# 2. UCS
# 3. Greedy Best First Search
# 4. A*

# Input.txt format:
# Line 1: Start End
# n following lines: Adjacency matrix(n x n)
# Command line: main.py input.txt < algorithm_name > (read comments in main.py for more information)

import numpy as np
from math import sqrt
from math import inf
from queue import PriorityQueue

def DFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        start node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO:  
    path=[]
    visited={}
    have_path = False
    stack = [start]
    node = [start]
    prev = start

    while (len(stack) > 0):
        i = stack.pop(len(stack) - 1)
        visited[i] = prev
        if (visited.get(end) != None):
            have_path = True
            break
        
        for j in range(len(matrix[i])):
            if (matrix[i][j] > 0 and (j not in node)):
                node.append(j)
                stack.append(j)
        prev = i
    
    if (have_path == False):
        return visited, path

    present = end
    for i in range(len(visited)):
        path.append(present)
        if (present == start):
            path.reverse()
            break
        present = visited.get(present)
        
    return visited, path

def BFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        start node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 
    path=[]
    visited={}
    have_path = False
    queue = [start]
    visited[start] = start

    while (len(queue) > 0):
        i = queue.pop(0)
        if (have_path == True):
            break
        
        for j in range(len(matrix[i])):
            if (matrix[i][j] > 0 and visited.get(j) == None):
                visited[j] = i
                queue.append(j)

            if (visited.get(end) != None):
                have_path = True
                break
        
    if (have_path == False):
        return visited, path

    present = end
    for i in range(len(visited)):
        path.append(present)
        if (present == start):
            path.reverse()
            break
        present = visited.get(present)

    return visited, path

def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        start node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    path = []
    visited = {}
    have_path = False
    prev = start
    queue = PriorityQueue()
    queue.put((0, start, prev))

    while (queue):
        cost, i, prev = queue.get()
        if (visited.get(i) == None):
            visited[i] = prev
            if i == end:
                have_path = True
                break
            for j in range(len(matrix)):
                if (matrix[i][j] > 0 and visited.get(j) == None):
                    total_cost = cost + matrix[i][j]
                    queue.put((total_cost, j, i))

    if (have_path == False):
        return visited, path

    present = end
    for i in range(len(visited)):
        path.append(present)
        if (present == start):
            path.reverse()
            break
        present = visited.get(present)

    return visited, path

def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        start node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path = []
    visited = {}
    have_path = False
    prev = start
    queue = PriorityQueue()
    queue.put((0, start, prev))

    while (queue):
        heuristic, i, prev = queue.get()
        if (visited.get(i) == None):
            visited[i] = prev
            if i == end:
                have_path = True
                break
            for j in range(len(matrix)):
                if (matrix[i][j] > 0 and visited.get(j) == None):
                    if (matrix[j][end] == 0 and j != end):
                        heuristic = inf
                    elif (matrix[j][end] == 0 and j == end):
                        heuristic = 0
                    elif (matrix[j][end] > 0):
                        heuristic = matrix[j][end]

                    queue.put((heuristic, j, i))

    if (have_path == False):
        return visited, path

    present = end
    for i in range(len(visited)):
        path.append(present)
        if (present == start):
            path.reverse()
            break
        present = visited.get(present)

    return visited, path

def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        start node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path = []
    visited = {}
    have_path = False
    prev = start
    queue = PriorityQueue()
    x1, y1 = pos.get(start)
    x2, y2 = pos.get(end)
    heuristic = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
    f = heuristic
    queue.put((f, 0, start, prev))

    while (queue):
        f, cost, i, prev = queue.get()
        if (visited.get(i) == None):
            visited[i] = prev
            if i == end:
                have_path = True
                break

            for j in range(len(matrix)):
                if (matrix[i][j] > 0 and visited.get(j) == None):
                    x1, y1 = pos.get(j)
                    x2, y2 = pos.get(end)
                    total_cost = cost + matrix[i][j]
                    heuristic = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
                    f = heuristic + total_cost
                    queue.put((f, total_cost, j, i))

    if (have_path == False):
        return visited, path

    present = end
    for i in range(len(visited)):
        path.append(present)
        if (present == start):
            path.reverse()
            break
        present = visited.get(present)

    return visited, path

