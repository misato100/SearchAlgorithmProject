import sys
import heapq
import queue
from array import *

search_type = str(sys.argv[1])
input_file = sys.argv[2]
initial = sys.argv[3] # The start state
goal = sys.argv[4] # The goal state
dist = [] # We add each distance between two cities in the path
sol = [] # The final path found by searching


# This finds children of a specific parent
# and returns a list of children
def find_children_for_fifo(parent):
    f = open("inputs.txt", "r")
    children = []
    position = 0
    # Read each line
    for path in f.readlines():
        words = path.split()
        # If parent is found on the line, then add it to the list
        if parent == words[0]:
            children.insert(position, [words[1], words[2]])
        elif words[0] == "END":
            return children
        elif parent == words[1]:
            children.insert(position, [words[0], words[2]])
        position = position + 1
    return children


# This finds a distance between two cities
def find_distance(city_1, city_2):
    f = open("inputs.txt", "r")
    distance = 0
    for path in f.readlines():
        words = path.split()
        if city_1 == words[0] and city_2 == words[1]:
            distance = words[2]
            return distance
        elif words[0] == "END":
            distance = -1
        elif city_1 == words[1] and city_2 == words[0]:
            distance = words[2]
            return distance
    return distance


# This returns an index of a last city in a list
def find_index(list, city):
    index = -1
    for cities in enumerate(list):
        if cities[len(cities) - 1] == city:
            index = cities[0]
    return index


# This allows to create a node with name and parent node
# This also includes total cost, heuristic distance and sum of both as well
class Node():
    # Initialize a Node
    def __init__(self, parent=None, name=None):
        self.parent = parent
        self.name = name

        self.g = 0
        self.h = 0
        self.f = 0

    # This shows that two Nodes are equal is their names are the same
    def __eq__(self, other):
        return self.name == other.name


# This allows to create a node with name and parent node and total cost
class UCS_Node():
    def __init__(self, g=None, parent=None, name=None):
        self.g = g
        self.parent = parent
        self.name = name

    def __eq__(self, other):
        return self.name == other.name

    # This allows to compare two nodes by their total costs
    def __lt__(self, other):
        return self.g < other.g


# This returns a list of children nodes from a specific parent node
def create_node_list(parent):
    children_nodes = []
    for child in find_children_for_fifo(parent.name):
        node = Node(parent, child[0])
        node.g = child[1]
        node.h = find_h(child[0])
        node.f = node.g + node.h
        children_nodes.append(node)
    return children_nodes


# This returns a list of children nodes from a specific parent node
def create_node_list_for_ucs(parent):
    children_nodes = []
    for child in find_children_for_fifo(parent[1].name):
        node = UCS_Node(child[1], parent, child[0])
        children_nodes.append(node)
    return children_nodes


# This returns a specified city's heuristic value
# by reading a text file
def find_h(city):
    h_value = -1
    f = open("heuristic_Frankfort.txt", "r")
    for line in f.readlines():
        words = line.split()
        if words[0] == city:
            h_value = words[1]
        elif words[0] == "END":
            return h_value
    return h_value


# This returns an index of a node with the cheapest value
# This function is mainly for A* search
def find_cheapest(list):
    cheapest_f = list[0].f
    index = 0
    for i, node in enumerate(list):
        if node.f < cheapest_f:
            index = i
    return index


# This returns the cheapest total cost of city in a list
def find_cheapest_ucs(list, city):
    index = -1
    for i, cities in enumerate(list):
        if cities == city:
            index = i
    return list[index].g


def ucs_function():
    solution = -1
    # Return if initial city is the same as the goal city
    if initial == goal:
        return solution
    start = UCS_Node(0, None, initial) # Start node
    end = UCS_Node(0, None, goal) # End node
    frontier = [] # Heap
    reached = []
    reached.append(start)
    heapq.heappush(frontier, (start.g, start)) # Add a data with a total cost and node

    # Keep looping until the heap is empty
    while (frontier):
        parent = heapq.heappop(frontier) # The cheapest node
        if solution > 0 and parent[0] < solution:
            break
        # Check each child node
        for child in create_node_list_for_ucs(parent):
            # Update the total cost
            new_cost = parent[0] + int(child.g)
            # Check if a child is not in a reached list or the total cost of the child is cheaper than the existing one
            if child not in reached or int(new_cost) < int(find_cheapest_ucs(reached, child)):
                child.g = new_cost
                child.parent = parent[1]
                reached.append(child)
                heapq.heappush(frontier, (new_cost, child))
                if child == end:
                    end.g = new_cost
                    end.parent = parent[1]
                    if child.g < int(find_cheapest_ucs(reached, child)):
                        solution = child.g
    # These lines below add the path and each distance to lists which were initialized in the beginning
    cn = end
    solution = temp = cn.g
    while cn is not None:
        sol.append(cn.name)
        cn = cn.parent
        if cn is not None:
            dist.append(temp - cn.g)
            temp = cn.g
    sol.reverse()
    dist.reverse()
    if len(sol) == 1:
        sol.clear()
    return solution


def bfs_function():
    solution = 0
    if initial == goal:
        solution = -1
        return solution
    frontier = queue.Queue() # This is a FIFO queue
    reached = []
    temp = 0
    found = False
    not_connected = True
    frontier.put(initial)

    while (not frontier.empty()):
        parent = frontier.get()
        sol.append(parent)
        found = False
        for child in find_children_for_fifo(parent):
            # Make the value as false because we found a connected city
            not_connected = False
            s = child[0]
            if s == goal:
                dist.append(child[1])
                sol.append(child[0])
                solution = int(solution) + int(child[1])
                return solution
            if found == False:
                if s not in reached:
                    reached.append(s)
                    frontier.put(s)
                    found = True
                    temp = child[1]
        not_connected = True
        dist.append(temp)
        solution = int(solution) + int(temp)
    # This is the case where there is no way of reaching to a goal from the start
    if not_connected == True:
        dist.clear()
        solution = 0
        sol.clear()
    return solution


def dfs_function():
    solution = 0
    if initial == goal:
        solution = -1
        return solution
    frontier = queue.LifoQueue() # This is LIFO queue
    reached = []
    temp = 0
    found = False
    not_connected = True
    frontier.put(initial)
    while (not frontier.empty()):
        parent = frontier.get()
        reached.append(parent)
        sol.append(parent)
        if int(temp) > 0:
            dist.append(temp)
        solution = int(solution) + int(temp)
        # If the picked value is the same as a goal state, then break the loop and return
        if parent == goal:
            return solution
        found = False
        for child in find_children_for_fifo(parent):
            if child[0] not in reached:
                frontier.put(child[0])
                temp = child[1]
                found = True
        if found == False:
            dist.pop(len(dist) - 1)
            solution = int(solution) - int(temp)
            sol.pop(len(sol) - 1)
            if frontier.qsize() > 0:
                temp = find_distance(frontier.queue[frontier.qsize() - 1], sol[len(sol) - 1])
    # If we reached these lines, that means start and goal states were not connected
    dist.clear()
    solution = 0
    sol.clear()
    return solution


def astar_function():
    solution = 0
    start = Node(None, initial) # Create a node with just an initial value
    start.g = start.h = start.f = 0 # Assign the total cost, heuristic value and the sum of both as 0
    end = Node(None, goal)
    end.g = end.h = end.f = 0
    open_list = []
    closed_list = []
    open_list.append(start)

    while (open_list):
        # If this function keeps adding elements to the open_list
        # and never returns any value, then break the loop here
        if len(open_list) > 1000:
            return solution
        # Find the current node we're checking right now
        current_i = find_cheapest(open_list)
        current_node = open_list[current_i]

        # This checks each node in open_list
        for i, data in enumerate(open_list):
            # Check if the sum of total cost and heuristic value is cheaper than current one
            if data.f < current_node.f:
                current_node = data
                current_i = i
        # After looping through the open_list, get the cheapest node from open_list
        # and add it to closed_list
        open_list.pop(current_i)
        closed_list.append(current_node)

        # If the current node is the goal, then add all the path and distance to a list
        # and reverse them to obtain the correct order
        if current_node == end:
            cn = current_node
            solution = temp = cn.g
            while cn is not None:
                sol.append(cn.name)
                cn = cn.parent
                if cn is not None:
                    dist.append(temp - cn.g)
                    temp = cn.g
            sol.reverse()
            dist.reverse()
            return solution

        # Check each child node of a current node
        for child in create_node_list(current_node):
            for child_closed in closed_list:
                # Check if the child is in a closed_list
                if child == child_closed:
                    continue
            child.g = current_node.g + int(child.g)
            child.h = find_h(child.name)
            child.f = child.g + int(child.h)
            for child_open in open_list:
                if child == child_open and child.g > child_open.g:
                    continue
            open_list.append(child)
    return solution


# This takes each search function as a parameter and print the output
def print_function(function):
    distance = function
    first = ""
    i = 0
    # If any path exists in a list, sol, then print everything
    if len(sol) > 0:
        print("distance: " + str(distance) + " mi")
        print("path:")
        for elem in sol:
            if first == "":
                first = elem
            else:
                print(first + " to " + elem + ": " + str(dist[i]) + " mi")
                i = i + 1
                first = elem
    # If distance is -1, then it means that the start and goal states are the same
    # So we just return 0
    elif distance == -1:
        print("distance: 0 mi")
        print("path: ")
        print("none")
    # If no path is found or distance is -1, then it means that the start state will never reach the goal state
    else:
        print("distance: infinity")
        print("path: ")
        print("none")


# This checks what type of search was typed
# (ucs/bfs/dfs/astar) or else it's an error
if search_type == "ucs":
    print_function(ucs_function())
elif search_type == "bfs":
    print_function(bfs_function())
elif search_type == "dfs":
    print_function(dfs_function())
elif search_type == "astar":
    print_function(astar_function())
else:
    print("ERROR!")
