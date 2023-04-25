import random
# Class representing an artificial ant of the ant colony
"""
    alpha: a parameter controlling the influence of the amount of pheromone during ants' path selection process
    beta: a parameter controlling the influence of the distance to the next node during ants' path selection process
"""
class Ant:
    def __init__(self, alpha: float, beta: float, initial_location):
        self.alpha = alpha
        self.beta = beta
        self.current_location = initial_location
        self.travelled_distance = 0
        self.visited_cities = [initial_location]

    # The ant runs to visit all the possible locations of the environment
    def run(self):
        while len(self.visited_cities) < len(self.environment.get_possible_locations()):
            next_city = self.select_path()
            distance = self.environment.get_distance(self.current_location, next_city)
            self.travelled_distance += distance
            self.current_location = next_city
            self.visited_cities.append(self.current_location)
        # Return to initial location
        distance = self.environment.get_distance(
            self.current_location, self.visited_cities[0]
        )
        self.travelled_distance += distance
        self.current_location = self.visited_cities[0]
        self.visited_cities.append(self.current_location)

    # Select the next path based on the random proportional rule of the ACO algorithm
    def select_path(self):
        feasible_neighborhood = list(
            set(self.environment.get_possible_locations()) - set(self.visited_cities)
        )
        probabilities = {}
        sum = 0
        for city in feasible_neighborhood:
            pheromone_value = self.environment.get_pheromone_value(
                self.current_location, city
            )
            distance = self.environment.get_distance(self.current_location, city)
            heuristic_distance = 1 / distance
            value = pheromone_value**self.alpha * heuristic_distance**self.beta
            probabilities[city] = value
            sum += value

        if sum == 0:
            return random.choice(list(probabilities.keys()))

        for city in feasible_neighborhood:
            probabilities[city] = probabilities[city] / sum

        next_city = random.choices(
            list(probabilities.keys()), list(probabilities.values())
        )[0]
        return next_city

    def reset(self, location):
        self.current_location = location
        self.travelled_distance = 0
        self.visited_cities = [location]

    # Position an ant in an environment
    def join(self, environment):
        self.environment = environment