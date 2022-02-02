# -*- coding: utf-8 -*-
"""
Created on Sat Sep 25 23:03:25 2021

@author: srevadig
"""

import numpy as np

visited_nodes = 0  # explored nodes
opened_nodes = 0  # expanded nodes

class Node():
# Constructor for node
    def __init__(self, state_of_node, move_direction, gn, hn, parent_node):
        self.state_of_node = state_of_node
        self.move_direction = move_direction
        self.gn = gn
        self.hn = hn
        self.parent_node = parent_node
        self.successor_directions = {"up": None,"left": None, "right": None, "down": None}

        
#Function to check if the tile can move left
    def left_check(self):
        w=np.where(self.state_of_node == 0)
        zero = w[0][0]
        # (0, 0), (1, 0), (2, 0) boundary for left move
        left_boundary=np.arange(0,7,3)
        if zero in left_boundary:  
            return None
        else:
            # swap the blank tile with right if not boundary
            changed_index=zero - 1
            interchanging_value = self.state_of_node[changed_index]
            next_state = self.state_of_node.copy()
            next_state[changed_index] = 0
            next_state[zero] = interchanging_value
            return next_state
        
        
   
  #Function to check if the tile can move up
      
        
    def up_check(self):
        w=np.where(self.state_of_node == 0)
        zero = w[0][0]
        # (0, 0), (0, 1), (0, 2l boundary for upward movement
        up_boundary=np.arange(3)
        if zero in up_boundary:
            return None
        else:
            # swap blank tile with down tile if not boundary
            changed_index=zero-3
            interchanging_value = self.state_of_node[changed_index]
            next_state = self.state_of_node.copy()
            next_state[changed_index] = 0
            next_state[zero] = interchanging_value
            return next_state
        
#Function to check if the tile can move right
    def right_check(self):
        w=np.where(self.state_of_node == 0)
        zero = w[0][0]
        #(0, 2), (1, 2), (2, 2) Boundary for right move
        right_boundary=np.arange(2,9,3)
        if zero in right_boundary:
            return None
        else:
            # swap blank tile with left tile,if not boundary 
            changed_index=zero + 1
            interchanging_value = self.state_of_node[changed_index]
            next_state = self.state_of_node.copy()
            next_state[zero + 1] = 0
            next_state[zero] = interchanging_value
            return next_state  

#Function to check if the tile can move downward
    def down_check(self):
        w=np.where(self.state_of_node == 0) 
        zero = w[0][0]
        # (2, 0), (2, 1), (2, 2)-Boundary for downward movement
        down_boundary=np.arange(6,9,1)
        if zero in down_boundary:
            return None
        else:
            # swap blank tile with up tile,if not boundary
            changed_index=zero + 3
            interchanging_value = self.state_of_node[changed_index]
            next_state = self.state_of_node.copy()
            next_state[changed_index] = 0
            next_state[zero] = interchanging_value
            return next_state
        
#Function to calculate heuristic using number of misplaced tiles
    def misplaced_heuristic(self, next_state, goal):
        ''' Function for counting misplaced tiles '''
        misplaced_tiles = 0
        for i in range(9):
            # if digit not in the grid position as that in the goal state_of_node, increase cost
            if next_state[i] != goal[i]:
                misplaced_tiles =misplaced_tiles+ 1
        return misplaced_tiles
    
#Function to calculate heuristic using manhattan distance
    def manhattan_distance(self, next_state, goal):
        ''' Function for calculating Manhattan distance '''
        current_node = next_state
        distance = 0
        goal_dict = {}
        for i in range(9):
            element=goal[i]
            goal_dict[element] = i

        for i in range(9):  
            # count distance 
            distance =distance+ abs(int(i/3) - int(goal_dict[current_node[i]]/3)) + abs(i % 3 - goal_dict[current_node[i]] % 3)
        return distance
    
#Function to select the type of heuristic the user wants and do work accordingly.
    def heuristic_function(self, next_state, goal, heuristic_function):
        if heuristic_function == "1":
            return self.misplaced_heuristic(next_state, goal)
        if heuristic_function == "2":
            return self.manhattan_distance(next_state, goal)
    
    def puzzle(self, goal, heuristic_function):
        # Priority queues for storing step costs
        step_costs = [(self, 0)]
        gn_queue = [(0, 0)]

        #visited_info_set maintains node information
        visited_info_set = set([])

        # Variables for counting explored nodes and explanded nodes
        global visited_nodes
        global opened_nodes
        visited_nodes = 0
        opened_nodes = 0

        while step_costs:
            #  priority queue
            step_costs = sorted(step_costs, key=lambda z: z[1])
            gn_queue = sorted(gn_queue, key=lambda z: z[1])

            # remove the current node from queue
            current_node = step_costs.pop(0)[0]
            current_gn = gn_queue.pop(0)[0]
            # append current node to visited_info_set
            tup=tuple(current_node.state_of_node.reshape(1, 9)[0])
            visited_info_set.add(tup)
            # increment  count of explored node
            visited_nodes =visited_nodes+ 1

            #  Print the path traversed to reach goal state,if the goal state is reached
            if np.array_equal(current_node.state_of_node, goal):
                current_node.print_path()
                return True
            else:
                #  expand nodes if goal state is not reached
                if current_node.up_check() is not None:
                    next_state = current_node.up_check()
                    if tuple(next_state.reshape(1, 9)[0]) not in visited_info_set:
                        gn = current_gn + 1
                        hn = self.heuristic_function(next_state, goal, heuristic_function)
                        fn = gn + hn
                        current_node.successor_directions["up"] = Node(
                            next_state, "up", gn, hn, current_node)
                        step_costs.append((current_node.successor_directions["up"], fn))
                        opened_nodes += 1
                        gn_queue.append((gn, fn))

                if current_node.left_check() is not None:
                    next_state = current_node.left_check()
                    if tuple(next_state.reshape(1, 9)[0]) not in visited_info_set:
                        gn = current_gn + 1
                        hn = self.heuristic_function(next_state, goal, heuristic_function)
                        fn = gn + hn
                        current_node.successor_directions["left"] = Node(next_state, "left", gn, hn, current_node)
                        step_costs.append((current_node.successor_directions["left"], fn))
                        opened_nodes = opened_nodes+1
                        gn_queue.append((gn, fn))

                if current_node.right_check() is not None:
                    next_state = current_node.right_check()
                    if tuple(next_state.reshape(1, 9)[0]) not in visited_info_set:
                        gn = current_gn + 1
                        hn = self.heuristic_function(next_state, goal, heuristic_function)
                        fn = gn + hn
                        current_node.successor_directions["right"] = Node(next_state, "right", gn, hn, current_node)
                        step_costs.append((current_node.successor_directions["right"], fn))
                        opened_nodes =opened_nodes+ 1
                        gn_queue.append((gn, fn))

                if current_node.down_check() is not None:
                    next_state = current_node.down_check()
                    if tuple(next_state.reshape(1, 9)[0]) not in visited_info_set:
                        gn = current_gn + 1
                        hn = self.heuristic_function(next_state, goal, heuristic_function)
                        fn = gn + hn
                        current_node.successor_directions["down"] = Node(next_state, "down", gn, hn, current_node)
                        step_costs.append((current_node.successor_directions["down"], fn))
                        opened_nodes =opened_nodes+1
                        gn_queue.append((gn, fn))

    def print_path(self):
        path = {"state_of_node": [self.state_of_node], "move_direction": [ self.move_direction], "gn": [self.gn], "hn": [self.hn]}

        while self.parent_node:
            self = self.parent_node
            path["state_of_node"].append(self.state_of_node)
            path["move_direction"].append(self.move_direction)
            path["gn"].append(self.gn)
            path["hn"].append(self.hn)

        step_count = 0
        while path["state_of_node"]:
            print("Step: ", step_count)
            print(path["state_of_node"].pop())
            print("Action: ", path["move_direction"].pop())
            print("Path cost(gn): ", path["gn"].pop())
            print("Heuristic cost(hn): ", path["hn"].pop())
            print()
            step_count =step_count+ 1

        print("Total Explored nodes: ", visited_nodes)
        print("Total Expanded nodes: ", opened_nodes)


def main():
#Take user input for initial state and goal state
    initial = input("Enter initial state separated by space:\n").split(' ')
    initial = np.asarray(initial, dtype=int)

    goal = input("Enter goal state separated by space:\n").split(' ')
    goal = np.asarray(goal, dtype=int)
    # Take user input to ask user's choice for heuristic type 
    heuristic_function = input("Press '1' for Misplaced Tiles AND '2' for Manhattan Distance.\n")
    #To initialize heuristic
    temp_node = Node(goal, None, 0, 0, None)
    heuristic_initialize = temp_node.heuristic_function(initial, goal, heuristic_function)
    # Tree initialization is done using root node
    root = Node(initial, None, 0, heuristic_initialize, None)
    root.puzzle(goal, heuristic_function)


main()
