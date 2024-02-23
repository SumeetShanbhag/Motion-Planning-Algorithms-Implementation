# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy.spatial import KDTree 

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path
        
    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        delta_x, delta_y = abs(p2[1] - p1[1]), abs(p2[0] - p1[0])
        step_x = 1 if p2[1] > p1[1] else -1
        step_y = 1 if p2[0] > p1[0] else -1
        x, y = p1[1], p1[0]
        error = delta_x - delta_y

        while x != p2[1] or y != p2[0]:
            if x < 0 or x >= self.size_col or y < 0 or y >= self.size_row:
                # If the path goes out of the map, consider it as collision
                return True
            if self.map_array[y][x] == 0:
                # If the cell is an obstacle, there is a collision
                return True

            error_2 = 2 * error
            if error_2 > -delta_y:
                error -= delta_y
                x += step_x
            if error_2 < delta_x:
                error += delta_x
                y += step_y

        return False


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        return np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Clear the graph
        self.graph.clear()
        
        # Generate sample points using uniform distribution
        sample_rows, sample_cols = self.sample_pts(n_pts, random=False)

        # Check for obstacles at each sample point
        for row, col in zip(sample_rows, sample_cols):
            if self.map_array[row][col] == 0:
                continue  # skip if obstacle present
            # Add valid point to self.samples
            self.samples.append((row, col))
    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Generate random points
        rows, cols = self.sample_pts(n_pts)
        # Check obstacle
        for row, col in zip(rows, cols):
            if self.map_array[row][col] == 1:
                self.samples.append((row, col))

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        # Generate random points
        rows_1, cols_1 = self.sample_pts(n_pts)

        # Generate random points at some distance from the preivous generated points
        scale = 10  # std
        delta = np.random.normal(0.0, scale, (n_pts, 2)).astype(int)
        rows_2 = rows_1 + delta[:, 0]
        cols_2 = cols_1 + delta[:, 1]

        # Check if the point is close to an obstacle
        for row1, col1, row2, col2 in zip(rows_1, cols_1, rows_2, cols_2):
            if not(0 <= row2 < self.size_row) or not(0 <= col2 < self.size_col):
                continue

            if self.map_array[row1][col1] == 1 and self.map_array[row2][col2] == 0:
                self.samples.append((row1, col1))
            elif self.map_array[row1][col1] == 0 and self.map_array[row2][col2] == 1:
                self.samples.append((row2, col2))


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Generate random points
        rows_1, cols_1 = self.sample_pts(n_pts)
        # Generate random points at some distance from the preivous generated points
        scale = 15 # std
        rows_2 = rows_1 + np.random.normal(0.0, scale, n_pts).astype(int)
        cols_2 = cols_1 + np.random.normal(0.0, scale, n_pts).astype(int)
        # check if it is the "bridge" form
        for row1, col1, row2, col2 in zip(rows_1, cols_1, rows_2, cols_2):
            # both are obstacles or outside the boundary
            if ((not(0 <= col2 < self.size_col) or not(0 <= row2 < self.size_row)) or \
                self.map_array[row2][col2] == 0) and \
                self.map_array[row1][col1] == 0:
                # make sure the midpoint is inside the map
                mid_row, mid_col = int(0.5*(row1+row2)), int(0.5*(col1+col2))
                if 0 <= mid_row < self.size_row and 0 <= mid_col < self.size_col and \
                    self.map_array[mid_row][mid_col] == 1:
                    self.samples.append((mid_row, mid_col))

    def sample_pts(self, n_pts, random=True):
        # number of points
        total_rows = int(np.sqrt(n_pts * self.size_row / self.size_col))
        total_cols = int(n_pts / total_rows)
        
        if random:
            # generate random points
            rows = np.random.randint(0, self.size_row, n_pts, dtype=int)
            cols = np.random.randint(0, self.size_col, n_pts, dtype=int)
        else:
            # generate uniform points
            sample_row = np.linspace(0, self.size_row-1, total_rows, dtype=int)
            sample_col = np.linspace(0, self.size_col-1, total_cols, dtype=int)
            rows, cols = np.meshgrid(sample_row, sample_col, indexing='ij')
            rows = rows.flatten()
            cols = cols.flatten()
        
        return rows[:n_pts], cols[:n_pts]

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()
    
    def add_weighted_edges(self, pairs):
        # Calculates weight and adds edges 
        for i, j in pairs:
            if i == "start":
                point1 = self.samples[-2]
            elif i == "goal":
                point1 = self.samples[-1]
            else:
                point1 = self.samples[i]
            point2 = self.samples[j]
            # checks for collision
            if self.check_collision(point1, point2):
                continue
            # calculates distance
            d = self.dis(point1, point2)
            self.graph.add_edge(i, j, weight=d)
                    
    def sample(self, n_pts=1000, sampling_method="uniform", num_neighbors=15):
        '''Coeighstruct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialization
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        #pairs = []

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.

        # Connect the samples
        self.kdtree = KDTree(list(self.samples))
        pairs = self.kdtree.query_pairs(num_neighbors)

        # Add the neighbor to graph
        self.graph.add_nodes_from(range(len(self.samples)))
        self.add_weighted_edges(pairs)
        
        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal, num_neighbors=15):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        
        start_goal_tree = KDTree([start, goal])
        neighbors = start_goal_tree.query_ball_tree(self.kdtree, num_neighbors)
        # list comprehensions 
        start_pairs = ([['start', neighbor] for neighbor in neighbors[0]])
        goal_pairs = ([['goal', neighbor] for neighbor in neighbors[1]])

        # Add the edge to graph
        self.add_weighted_edges(start_pairs)
        self.add_weighted_edges(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)

