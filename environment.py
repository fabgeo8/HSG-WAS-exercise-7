import math
import tsplib95
# Class representing the environment of the ant colony
"""
    rho: pheromone evaporation rate
"""
class Environment:
    def __init__(self, rho, m):

        self.rho = rho

        # ant population
        self.m = m

        # Initialize the environment topology
        self.tsp_problem = tsplib95.load('./att48-specs/att48.tsp')

        # get all edges (possible connections/paths)
        self.tsp_problem.get_edges()

        self.nodes = self.tsp_problem.get_nodes()

        self.edges = self.tsp_problem.get_edges()

        self.dimension = self.tsp_problem.dimension
        self.coordinates = list(self.tsp_problem.node_coords.values())

        # Initialize the pheromone map in the environment
        self.pheromone = []

        self.initialize_pheromone_map()

    # Intialize the pheromone trails in the environment
    def initialize_pheromone_map(self):
       dimension = self.tsp_problem.dimension
       init_value = self._get_initial_pheromone_value()
       self.pheromone_map = [[init_value] * dimension for _ in range(dimension)]

    # Update the pheromone trails in the environment
    def update_pheromone_map(self, ants):
        # Evaporate pheromones
        for i, row in enumerate(self.pheromone):
            for j, col in enumerate(row):
                self.pheromone[i][j] *= (1-self.rho)

        # Add new pheromone
        for ant in ants:
            for x in range(len(ant.path)):
                delta_tau = 1 / ant.travelled_distance
                i, j = ant.path[x-1] - 1, ant.path[x] - 1
                self.pheromone[i][j] += delta_tau
                self.pheromone[j][i] += delta_tau

    # Get pheromone value of trail in the environment
    def get_pheromone_value(self, current_node, next_node):
        return self.pheromone_map[current_node - 1][next_node - 1]

    def get_distance(self, current_node, next_node):
        edge = current_node, next_node
        return self.tsp_problem.get_weight(*edge)

    # Get the pheromone trails in the environment
    def get_pheromone_map(self):
        return self.pheromone
    
    # Get the initial pheromone default value
    def _get_initial_pheromone_value(self):
        nodes = list(self.tsp_problem.get_nodes())
        cnn_sum = 0
        for i in range(len(nodes)):
            nodes_copy = nodes.copy()
            first = nodes_copy.pop(i)
            cnn = self._cnn(first, nodes_copy, 0)
            cnn_sum += cnn
        
        avg_cnn = int(cnn_sum / len(nodes))

        init_value = self.m / avg_cnn
        print(f"Computed initial pheromone value: {init_value}")
        print(f"Ant count: {self.m} / average Cnn: {avg_cnn}")
        return init_value
    
    # Update the pheromone trails in the environment
    def update_pheromone_map(self, tours):
        # Evaporation of pheromone
        updated_map = []
        for row in self.pheromone_map:
            updated_row = []
            for value in row:
                updated_value = value * (1 - self.rho)
                updated_row.append(updated_value)
            updated_map.append(updated_row)

        # Adding of pheromones
        for tour in tours:
            # Compute total cost of tour
            cost = 0
            for i in range(len(tour) - 1):
                weight = self.get_distance(tour[i], tour[i + 1])
                cost += weight

            pheromone_amount = 1 / cost

            # Add pheromone to map
            for i in range(len(tour) - 1):
                # Adjusted for map indexing at zero
                map_coords = tour[i] - 1, tour[i + 1] - 1
                updated_map[map_coords[0]][map_coords[1]] = (
                    updated_map[map_coords[0]][map_coords[1]] + pheromone_amount
                )
                # Update both directions
                updated_map[map_coords[1]][map_coords[0]] = (
                    updated_map[map_coords[1]][map_coords[0]] + pheromone_amount
                )

        self.pheromone_map = updated_map
    
    # Get the environment topology
    def get_possible_locations(self):
        return list(self.tsp_problem.get_nodes())

    def _cnn(self, current_node, remaining_nodes, cost):
        if len(remaining_nodes) == 1:
            distance = self.get_distance(current_node, remaining_nodes[0])
            return cost + distance
        else:
            min_distance = None
            for node in remaining_nodes:
                distance = self.get_distance(current_node, node)
                if min_distance == None:
                    min_distance = (node, distance)
                else:
                    if distance < min_distance[1]:
                        min_distance = (node, distance)

            current_node = min_distance[0]
            remaining_nodes.remove(current_node)
            cost += min_distance[1]
            return self._cnn(current_node, remaining_nodes, cost)