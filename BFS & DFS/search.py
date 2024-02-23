# Basic searching algorithms
# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

def valid_neighbours(node, grid, set_parent=True):
    row, col = node.row, node.col
    neighbours = [(row, col+1), (row+1, col), (row, col-1), (row-1, col)]
    Neighbours = []
    
    for neighbour in neighbours:
        row, col = neighbour
        if 0 <= row < len(grid) and 0 <= col < len(grid[0]) and grid[row][col] != 1:
            Neighbours.append(Node(row, col, 0, 0))
    
    if set_parent:
        for neighbour in Neighbours:
            neighbour.parent = node
    
    return Neighbours

def nodeExists(node, node_list):
    """ Returns true if a node's coordinate is found in a list of nodes """
    for i in node_list:
        if (i.row, i.col) == (node.row, node.col):
            return True
    return False

def DFS(visited_nodes, grid, node, step, goal, found):
    if not nodeExists(node, visited_nodes):
        visited_nodes.append(node)
        step += 1
        if [node.row, node.col] == goal:
            found = True
            path = []
            while node.parent:
                path.insert(0, [node.row, node.col])
                node = node.parent
            path.insert(0, [node.row, node.col])  # Starting point was not included in path
            return [found, path, step]
        for neighbour in valid_neighbours(node, grid):
            found, path, step = DFS(visited_nodes, grid, neighbour, step, goal, found)
            if found:
                return [found, path, step]
    return [found, [], step]

def bfs(grid, start, goal):
    '''
    Return a path found by BFS alogirhm and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###    
    path = []
    steps = 0
    found = False

    visited = []
    queue = []
    start_node = Node(start[0], start[1], grid[start[0]][start[1]], 0)
    visited.append(start_node)
    steps += 1
    queue.append(start_node)

    while queue:
        if found:
            break

        current_node = queue.pop(0)
        for neighbour in valid_neighbours(current_node, grid):
            if not nodeExists(neighbour, visited):
                steps += 1
                visited.append(neighbour)
                if [neighbour.row, neighbour.col] == goal:
                    found = True
                    while neighbour.parent:
                        path.insert(0, [neighbour.row, neighbour.col])
                        neighbour = neighbour.parent
                    path.insert(0, [neighbour.row, neighbour.col])
                    break
                queue.append(neighbour)

    if found:
        print(f"It takes {steps} steps to find a path using BFS",path)
    else:
        print("No path found")

    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    visited = []
    Start = Node(start[0], start[1], grid[start[0]][start[1]],0)

    [found, path, steps] = DFS(visited, grid, Start, steps, goal, found)

    if found:
        print(f"It takes {steps} steps to find a path using DFS",path)
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
