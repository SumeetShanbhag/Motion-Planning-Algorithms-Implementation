# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
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
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    rows = len(grid)
    cols = len(grid[0])
    nodes = []
    for i in range(rows):
        for j in range(cols):
            nodes.append(Node(i, j, grid[i][j], 0))

    start_node = nodes[start[0]*cols + start[1]]
    goal_node = nodes[goal[0]*cols + goal[1]]
    start_node.g = 0

    heap = [start_node]
    while len(heap) > 0:
        curr_node = heap.pop(0)
        if curr_node.row == goal_node.row and curr_node.col == goal_node.col:
            found = True
            goal_node.parent = curr_node.parent
            break
        for i, j in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_row = curr_node.row + i
            new_col = curr_node.col + j
            if new_row >= 0 and new_row < rows and new_col >= 0 and new_col < cols:
                new_node = nodes[new_row*cols + new_col]
                if new_node.is_obs == 0:
                    new_g = curr_node.g + 1
                    if i != 0 and j != 0: # for diagonal moves
                        new_g += 0.5
                    if new_node.g is None or new_g < new_node.g:
                        new_node.g = new_g
                        new_node.parent = curr_node
                        heap.append(new_node)
        steps += 1

    if found:
        node = goal_node
        while node is not None:
            path.append([node.row, node.col])
            node = node.parent
        path.reverse()
        
        print(f"It takes {steps-1} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps-1


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
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
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    rows = len(grid)
    cols = len(grid[0])
    nodes = []
    for i in range(rows):
        for j in range(cols):
            nodes.append(Node(i, j, grid[i][j], abs(i-goal[0]) + abs(j-goal[1]))) # using manhattan distance as heuristic

    start_node = nodes[start[0]*cols + start[1]]
    goal_node = nodes[goal[0]*cols + goal[1]]
    start_node.g = 0
    start_node.cost = start_node.g + start_node.h

    heap = [start_node]
    while len(heap) > 0:
        curr_node = heap.pop(0)
        #steps += 1
        if curr_node.row == goal_node.row and curr_node.col == goal_node.col:
            found = True
            goal_node.parent = curr_node.parent
            break
        for i, j in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_row = curr_node.row + i
            new_col = curr_node.col + j
            if new_row >= 0 and new_row < rows and new_col >= 0 and new_col < cols:
                new_node = nodes[new_row*cols + new_col]
                if new_node.is_obs == 0:
                    new_g = curr_node.g + 1
                    if i != 0 and j != 0: # for diagonal moves
                        new_g += 0.5
                    if new_node.g is None or new_g < new_node.g:
                        new_node.g = new_g
                        new_node.cost = new_g + new_node.h
                        new_node.parent = curr_node
                        heap.append(new_node)
        heap = sorted(heap, key=lambda x: x.cost) # sort heap based on cost (f = g + h)
        steps += 1
    if found:
        node = goal_node
        while node is not None:
            path.append([node.row, node.col])
            node = node.parent
        path.reverse()
    
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    print(path)
    return path, steps
    # if found:
    #     print(f"It takes {steps} steps to find a path using A*")
    # else:
    #     print("No path found")
    # return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()

