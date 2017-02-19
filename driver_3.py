import sys, time
                   
class Report_form:
    def commit_report(self):
        with open('output.txt', 'w') as report_file:
            report_file.write(self.path)
            report_file.write(self.path_cost)
            report_file.write(self.expanded_nodes)
            report_file.write(self.fringe_size)
            report_file.write(self.max_fringe_size)
            report_file.write(self.search_depth)
            report_file.write(self.max_search_depth)
            report_file.write(self.running_time + str(round(time.clock() - self.start_time, 8)) + '\n')
            report_file.write(self.ram_usage)
    def __init__(self):
        self.start_time = time.clock()
        self.path = ''
        self.path_cost = ''
        self.expanded_nodes = ''
        self.fringe_size = ''
        self.max_fringe_size = ''
        self.search_depth = ''
        self.max_search_depth = ''
        self.running_time = 'running_time: '
        self.ram_usage = 'max_ram_usage: Need to set up a Linux VM'
    
class Vertex:
    def Hamming_distance(self, state, goal):
        return sum([int(state[i] != goal[i]) for i in range(0, len(state)) if state[i] != 0])
    def Manhattan_distance(self, state, goal):
        return int(sum([abs(i % self.grid_size - goal.index(state[i]) % self.grid_size) + abs(i // self.grid_size - goal.index(state[i]) // self.grid_size) for i in range(len(state)) if state[i] not in (0, goal[i])]))
    def Set_cost_traversed(self, cost):
        self.traversed_cost = cost
        self.total_cost = self.traversed_cost + self.estimated_cost
    def get_neighbours(self):
        neighbours = [[],[],[],[]]
        for i in [-1,1]:
            if self.state.index(0) + self.grid_size * i in range(len(self.state)):
                neighbour = self.state[:]
                a = neighbour.index(0)
                b = neighbour.index(0) + self.grid_size * i
                neighbour[a], neighbour[b] = neighbour[b], neighbour[a]
                neighbours[(i + 2) // 2] = neighbour
            if (self.state.index(0) + i) in range(len(self.state)) and self.state.index(0) // self.grid_size == (self.state.index(0) + i) // self.grid_size:
                neighbour = self.state[:]
                a = neighbour.index(0)
                b = neighbour.index(0) + i
                neighbour[a], neighbour[b] = neighbour[b], neighbour[a]
                neighbours[(i + 6) // 2] = neighbour
        neighbours = [i for i in neighbours if i != []]
        if self.mode == 'DFS':
            neighbours.reverse()
        return neighbours
    def __init__(self, state, parent = None, depth = 0, mode = 'BFS', traversed_cost = 0, goal = None):
        self.mode = mode
        self.state = state
        self.grid_size = int(len(self.state) ** 0.5)
        self.depth = depth
        self.parent = parent
        self.neighbours = self.get_neighbours()
        self.traversed_cost = traversed_cost
        self.estimated_cost = 0 if self.mode != 'AST' else self.Manhattan_distance(self.state, goal)
        self.total_cost = self.traversed_cost + self.estimated_cost

def Args_valid(args_list):
    def state_solvable(initial_state):
        grid_size = int(len(initial_state) ** 0.5)
        row_oddity = ((initial_state.index(0) // grid_size + 2) % 2 == 1)
        oddities = 0
        initial_state.remove(0)
        for i in range(len(initial_state)):
            for j in range(i+1, len(initial_state)):
                if initial_state[i] > initial_state[j]:
                    oddities += 1
        if grid_size % 2 == 1:
            return ((oddities % 2) == 0)
        return ((oddities % 2) == 0 and row_oddity) or ((oddities % 2) == 1 and not row_oddity)
    error_message = ''
    if len(args_list) != 2:
        error_message = 'The script is supposed to be executed like this: driver.py <search method> <puzzle description>'
        return error_message
    method = args_list[0].upper()
    if method not in ('BFS','DFS','AST','IDA'):
        error_message = 'Search method not specified. Valid search methods are BFS, DFS, AST and IDA'
        return error_message
    initial_state = []
    for i in args_list[1].split(','):
        try:
            int(i)
            initial_state.append(int(i))
        except ValueError:
            error_message = ('Puzzle description should contain only numbers and commas')
            return error_message
    if len(initial_state) != len(set(initial_state)):
        error_message = 'Puzzle description should contain only unique numbers'
        return error_message
    if len(initial_state)**0.5 != float(int(len(initial_state)**0.5)):
        error_message = 'Puzzle description should represent a square'
        return error_message
    if int(len(initial_state)*(max(initial_state)/2)) != sum(initial_state):
        error_message = 'Puzzle description should contain only consecutive numbers starting from 0'
    if not state_solvable(initial_state):
        error_message = 'The given puzzle is not solvable'
        return error_message
    return error_message
        
def Bfs(root, goal, report):
    queue = [Vertex(root, None, 0, 'BFS')]
    explored = []
    visited = set(tuple(explored))
    expanded_nodes = 0
    parent = {tuple(root) : None}
    max_queue = 1
    max_depth = 0
    while queue:
        current_vertex = queue.pop(0)
        if current_vertex.state == goal:
            directions = {}
            directions[-1 * current_vertex.grid_size] = 'Up'
            directions[current_vertex.grid_size] = 'Down'
            directions[-1] = 'Left'
            directions[1] = 'Right'
            route = []
            route.append(parent[tuple(goal)])
            while route[-1] != root:
                route.append(parent[tuple(route[-1])])
            route.reverse()
            route.append(goal)
            path = []
            for j in range(1, len(route)):
                direction = route[j].index(0) - route[j-1].index(0)
                path.append(directions[direction])
            report.path = 'path_to_goal: ' + str(path) + '\n'
            report.path_cost = 'cost_of_path: ' + str(len(path)) + '\n'
            report.expanded_nodes = 'nodes_expanded: ' + str(expanded_nodes) + '\n'
            report.fringe_size = 'fringe_size: ' + str(len(queue)) + '\n'
            report.max_fringe_size = 'max_fringe_size: ' + str(max_queue) + '\n'
            report.search_depth = 'search_depth: ' + str(current_vertex.depth) + '\n'
            report.max_search_depth = 'max_search_depth: ' + str(max_depth) + '\n'
            return True
        explored.append(current_vertex)
        visited.add(tuple(current_vertex.state))
        expanded_nodes += 1
        for i in current_vertex.neighbours:
            if tuple(i) not in visited:
                parent[tuple(i)] = current_vertex.state
                queue.append(Vertex(i, current_vertex.state, current_vertex.depth + 1, 'BFS'))
                if queue[-1].depth > max_depth:
                    max_depth = queue[-1].depth
                if max_queue < len(queue):
                    max_queue = len(queue)
    return False

def Dfs(root, goal, report):
    stack = [Vertex(root, None, 0, 'DFS')]
    explored = set()
    backyard = set(root)
    expanded_nodes = 0
    parent = {tuple(root) : None}
    max_stack = 1
    max_depth = 0
    while stack:
        current_vertex = stack.pop()
        explored.add(tuple(current_vertex.state))
        if current_vertex.state == goal:
            directions = {}
            directions[-1 * current_vertex.grid_size] = 'Up'
            directions[current_vertex.grid_size] = 'Down'
            directions[-1] = 'Left'
            directions[1] = 'Right'
            route = []
            route.append(parent[tuple(goal)])
            while route[-1] != root:
                route.append(parent[tuple(route[-1])])
            route.reverse()
            route.append(goal)
            path = []
            for j in range(1, len(route)):
                direction = route[j].index(0) - route[j-1].index(0)
                path.append(directions[direction])
            report.path = 'path_to_goal: ' + str(path) + '\n'
            report.path_cost = 'cost_of_path: ' + str(len(path)) + '\n'
            report.expanded_nodes = 'nodes_expanded: ' + str(expanded_nodes) + '\n'
            report.fringe_size = 'fringe_size: ' + str(len(stack)) + '\n'
            report.max_fringe_size = 'max_fringe_size: ' + str(max_stack) + '\n'
            report.search_depth = 'search_depth: ' + str(current_vertex.depth) + '\n'
            report.max_search_depth = 'max_search_depth: ' + str(max_depth) + '\n'
            return True
        expanded_nodes += 1
        for neighbour in current_vertex.neighbours:
            if tuple(neighbour) not in explored and tuple(neighbour) not in backyard:
                parent[tuple(neighbour)] = current_vertex.state
                backyard.add(tuple(neighbour))
                stack.append(Vertex(neighbour, current_vertex.state, current_vertex.depth + 1, 'DFS'))
                if stack[-1].depth > max_depth:
                    max_depth = stack[-1].depth
                if max_stack < len(stack):
                    max_stack = len(stack)
    return False

def Select_minimum_cost_vertex(open_set, goal):
    return next((i for i, item in enumerate(open_set) if item.total_cost == min([i.total_cost for i in open_set])), -1)

def Determine_heritage(explored, state):
    heritage = [state.state]
    ancestor = tuple(state.state)
    while explored[ancestor].parent != None:
        heritage.append(explored[ancestor].parent)
        ancestor = tuple(explored[ancestor].parent)
    heritage.reverse()
    return heritage
    
def A_star(root, goal, report):
    open_set = [Vertex(root, None, 0, 'AST', 0, goal)]
    closed_set = []
    temp_set = {}
    expanded_nodes = 0
    max_fringe = 1
    while open_set:
        current_vertex = open_set.pop(Select_minimum_cost_vertex(open_set, goal))
        closed_set.append(current_vertex)
        expanded_nodes += 1
        temp_set[tuple(current_vertex.state)]=current_vertex
        if current_vertex.state == goal:
            directions = {}
            directions[-1 * current_vertex.grid_size] = 'Up'
            directions[current_vertex.grid_size] = 'Down'
            directions[-1] = 'Left'
            directions[1] = 'Right'
            route = Determine_heritage(temp_set, current_vertex)
            path = []
            for j in range(1, len(route)):
                direction = route[j].index(0) - route[j-1].index(0)
                path.append(directions[direction])
            report.path = 'path_to_goal: ' + str(path) + '\n'
            report.path_cost = 'cost_of_path: ' + str(len(path)) + '\n'
            report.expanded_nodes = 'nodes_expanded: ' + str(expanded_nodes) + '\n'
            report.fringe_size = 'fringe_size: ' + str(len(open_set)) + '\n'
            report.max_fringe_size = 'max_fringe_size: ' + str(max_fringe) + '\n'
            report.search_depth = 'search_depth: ' + str(current_vertex.depth) + '\n'
            report.max_search_depth = 'max_search_depth: ' + str(max([k.depth for k in closed_set])) + '\n'
            return Determine_heritage(temp_set, current_vertex)
        for neighbour in current_vertex.neighbours:
            if tuple(neighbour) not in temp_set: 
                temp_set[tuple(neighbour)] = Vertex(neighbour, current_vertex.state, current_vertex.depth + 1, 'AST', current_vertex.traversed_cost + 1, goal)
            if temp_set[tuple(neighbour)] not in closed_set:
                if temp_set[tuple(neighbour)] not in open_set:
                    open_set.append(Vertex(neighbour, current_vertex.state, current_vertex.depth + 1, 'AST', current_vertex.traversed_cost + 1, goal))
                    if len(open_set) > max_fringe:
                        max_fringe = len(open_set)
                else:
                    if current_vertex.traversed_cost < open_set[next((i for i, item in enumerate(open_set) if item.state == current_vertex.state), -1)].traversed_cost:
                        open_set[next((i for i, item in enumerate(open_set) if item.state == current_vertex.state), -1)].parent = current_vertex.state
                        open_set[next((i for i, item in enumerate(open_set) if item.state == current_vertex.state), -1)].Set_cost_traversed(current_vertex.traversed_cost + 1)
    return False

def ID_A_Star(root, goal, report):
    return False
        
def Main():
    error_message = Args_valid(sys.argv[1:])
    if error_message != '':
        print(error_message)
        sys.exit()
    root = [int(i) for i in sys.argv[2].split(',')]
    goal = list(range(len(root)))
    report = Report_form()
    if sys.argv[1].upper() == 'IDA':
        ID_A_Star(root, goal, report)
    elif sys.argv[1].upper() == 'AST':
        A_star(root, goal, report)
    elif sys.argv[1].upper() == 'DFS':
        Dfs(root, goal, report)
    else:
        Bfs(root, goal, report)
    report.commit_report()

if __name__ == "__main__":
    Main()