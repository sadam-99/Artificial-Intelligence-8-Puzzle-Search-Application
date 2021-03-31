# <!-- // Code Author:-
# // Name: Shivam Gupta
# // Net ID: SXG190040
# // Artificial Intelligence (CS 6364.001) Assignment 1 (Search Algorithms) -->

import numpy as np
class Search_Node():
    def __init__(self, state, parent, movement, depth, step_cost, path_cost, heuristic_cost):

        # children nodes are the nodes itself
        
        self.transfer_node_left = None
        self.transfer_node_right = None
        self.transfer_node_up = None
        self.transfer_node_down = None
        
        self.state = state
        self.parent = parent
        self.movement = movement
        self.depth = depth
        self.step_cost = step_cost
        self.path_cost = path_cost
        self.heuristic_cost = heuristic_cost

    def Transfer_UP(self):
        """[This function moves the value below the tile towards Up]

        Returns:
            [type]: [New State, Down Value]
        """
        Empty_Tile_index = [i[0] for i in np.where(self.state == 0)]
        if Empty_Tile_index[0] == 2:
            return False
        else:
            lower_Node_Value = self.state[Empty_Tile_index[0] + 1, Empty_Tile_index[1]]  # value of the lower tile
            Output_state = self.state.copy()
            Output_state[Empty_Tile_index[0], Empty_Tile_index[1]] = lower_Node_Value
            Output_state[Empty_Tile_index[0] + 1, Empty_Tile_index[1]] = 0
            return Output_state, lower_Node_Value

    def Transfer_DOWN(self):
        """[This function moves the value above the tile towards Downwards]

        Returns:
            [type]: [New State, Upper Value]
        """
        Empty_Tile_index = [i[0] for i in np.where(self.state == 0)]
        if Empty_Tile_index[0] == 0:
            return False
        else:
            upper_Node_Value = self.state[Empty_Tile_index[0] - 1, Empty_Tile_index[1]]  # value of the upper tile
            Output_state = self.state.copy()
            Output_state[Empty_Tile_index[0], Empty_Tile_index[1]] = upper_Node_Value
            Output_state[Empty_Tile_index[0] - 1, Empty_Tile_index[1]] = 0
            return Output_state, upper_Node_Value

    def Transfer_LEFT(self):
        """[This function moves the value right of the tile towards Left]

        Returns:
            [type]: [New State, Right Value]
        """
        Empty_Tile_index = [i[0] for i in np.where(self.state == 0)]
        if Empty_Tile_index[1] == 2:
            return False
        else:
            right_Node_Value = self.state[Empty_Tile_index[0], Empty_Tile_index[1] + 1]  # value of the right tile
            Output_state = self.state.copy()
            Output_state[Empty_Tile_index[0], Empty_Tile_index[1]] = right_Node_Value
            Output_state[Empty_Tile_index[0], Empty_Tile_index[1] + 1] = 0
            return Output_state, right_Node_Value

    def Transfer_RIGHT(self):
        """[This function moves the value left of the tile towards Right]

        Returns:
            [type]: [New State, Left Value]
        """
        Empty_Tile_index = [i[0] for i in np.where(self.state == 0)]
        if Empty_Tile_index[1] == 0:
            return False
        else:
            left_Node_Value = self.state[Empty_Tile_index[0], Empty_Tile_index[1] - 1]  # value of the left tile
            Output_state = self.state.copy()
            Output_state[Empty_Tile_index[0], Empty_Tile_index[1]] = left_Node_Value
            Output_state[Empty_Tile_index[0], Empty_Tile_index[1] - 1] = 0
            return Output_state, left_Node_Value


    def evaluate_Heuristic_Cost(self, Output_state, GOAL_state, heuristic_technique):
        """[This function calculates the heuristic Value based on the type of heuristic technique]

        Args:
            Output_state ([type]): [The state]
            GOAL_state ([type]): [Final State]
            heuristic_function ([type]): [type of Heuristic function]

        Returns:
            [type]: [description]
        """
        if heuristic_technique == 'num_misplaced':
            return self.evaluate_Astar1_misplacednum_Cost(Output_state, GOAL_state)
        elif heuristic_technique == 'manhattan':
            return self.evaluate_Astar2_manhatten_Cost(Output_state, GOAL_state)

    def evaluate_Astar1_misplacednum_Cost(self, current_state, GOAL_state):
        """[summary]

        Args:
            current_state ([type]): [The state]
            GOAL_state ([type]): [Final state]

        Returns:
            [type]: [Returns the Cost using No. of Misplaced Technique]
        """
        # For excluding the empty tile
        misplaced_cost = np.sum(current_state != GOAL_state) - 1  
        if misplaced_cost > 0:
            return misplaced_cost
        else:
            return 0  # when all tiles matches


    def evaluate_Astar2_manhatten_Cost(self, current_state, GOAL_state):
        """[summary]

        Args:
            current_state ([type]): [The state]
            GOAL_state ([type]): [Final state]

        Returns:
            [type]: [Returns the Cost using Manhatten Technique]
        """
        # Mapping for the Numbers and postions in the final state
        Goal_Mapping = {1: (0, 0), 2: (0, 1), 3: (0, 2), 4: (1, 0), 5: (1, 1), 6: (1, 2), 7: (2, 0), 8: (2, 1),
                             0: (2, 2)}
        Mahatten_Cost = 0
        for i in range(3):
            for j in range(3):
                if current_state[i, j] != 0:
                    Mahatten_Cost += sum(abs(p - q) for p, q in zip((i, j), Goal_Mapping[current_state[i, j]]))
        return Mahatten_Cost
    def Update_Nodes(self,Current_Node, Output_state,Visited_Stack_Set,Node_VALUE, action_STR,Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost ):

        # check if the node is already Visited_Stack_Set or not
        if tuple(Output_state.reshape(1, 9)[0]) not in Visited_Stack_Set:
        # create a new child node
            if action_STR == "right":
                Current_Node.transfer_node_right = Search_Node(state=Output_state, parent=Current_Node, movement=action_STR,
                                        depth=Current_Depth + 1,
                                        step_cost=Node_VALUE, path_cost=Current_path_COST + Node_VALUE,
                                        heuristic_cost=0)
                if Current_Node.transfer_node_right.depth <= 10:
                                UNVISITED_Stack.append(Current_Node.transfer_node_right)
                                Stack_Depth.append(Current_Depth + 1)
                                Stack_Path_cost.append(Current_path_COST + Node_VALUE)
            elif action_STR == "left":
                Current_Node.transfer_node_left = Search_Node(state=Output_state, parent=Current_Node, movement=action_STR,
                                        depth=Current_Depth + 1,
                                        step_cost=Node_VALUE, path_cost=Current_path_COST + Node_VALUE,
                                        heuristic_cost=0)
                if Current_Node.transfer_node_left.depth <= 10:
                                UNVISITED_Stack.append(Current_Node.transfer_node_left)
                                Stack_Depth.append(Current_Depth + 1)
                                Stack_Path_cost.append(Current_path_COST + Node_VALUE)
            elif action_STR == "up":
                Current_Node.transfer_node_up = Search_Node(state=Output_state, parent=Current_Node, movement=action_STR,
                                        depth=Current_Depth + 1,
                                        step_cost=Node_VALUE, path_cost=Current_path_COST + Node_VALUE,
                                        heuristic_cost=0)
                if Current_Node.transfer_node_up.depth <= 10:
                                UNVISITED_Stack.append(Current_Node.transfer_node_up)
                                Stack_Depth.append(Current_Depth + 1)
                                Stack_Path_cost.append(Current_path_COST + Node_VALUE)
            elif action_STR == "down":
                Current_Node.transfer_node_down = Search_Node(state=Output_state, parent=Current_Node, movement=action_STR,
                                        depth=Current_Depth + 1,
                                        step_cost=Node_VALUE, path_cost=Current_path_COST + Node_VALUE,
                                        heuristic_cost=0)
                if Current_Node.transfer_node_down.depth <= 10:
                                UNVISITED_Stack.append(Current_Node.transfer_node_down)
                                Stack_Depth.append(Current_Depth + 1)
                                Stack_Path_cost.append(Current_path_COST + Node_VALUE)
            
        return Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost 

    def Update_Nodes_AStar_Search(self,Current_Node, Output_state,Visited_Stack_Set,Node_VALUE, action_STR,Current_Depth,Current_path_COST,unvisited_q,Stack_Depth,Stack_Path_cost, GOAL_state, heuristic_function):

        # check if the node is already Visited_Stack_Set or not
        if tuple(Output_state.reshape(1, 9)[0]) not in Visited_Stack_Set:
            path_cost = Current_path_COST + Node_VALUE
            depth = Current_Depth + 1
        # get heuristic cost
            h_cost = self.evaluate_Heuristic_Cost(Output_state, GOAL_state, heuristic_function)
            total_cost = path_cost + h_cost
        # create a new child node
            if action_STR == "right":
                Current_Node.transfer_node_right = Search_Node(state=Output_state, parent=Current_Node, movement=action_STR, depth=depth, \
                                                      step_cost=Node_VALUE, path_cost=path_cost, heuristic_cost=h_cost)
                unvisited_q.append((Current_Node.transfer_node_right, total_cost))
                Stack_Depth.append((depth, total_cost))
                Stack_Path_cost.append((path_cost, total_cost))
            elif action_STR == "left":
                Current_Node.transfer_node_left = Search_Node(state=Output_state, parent=Current_Node, movement=action_STR, depth=depth, \
                                                      step_cost=Node_VALUE, path_cost=path_cost, heuristic_cost=h_cost)
                unvisited_q.append((Current_Node.transfer_node_left, total_cost))
                Stack_Depth.append((depth, total_cost))
                Stack_Path_cost.append((path_cost, total_cost))
            elif action_STR == "up":
                Current_Node.transfer_node_up = Search_Node(state=Output_state, parent=Current_Node, movement=action_STR, depth=depth, \
                                                      step_cost=Node_VALUE, path_cost=path_cost, heuristic_cost=h_cost)
                unvisited_q.append((Current_Node.transfer_node_up, total_cost))
                Stack_Depth.append((depth, total_cost))
                Stack_Path_cost.append((path_cost, total_cost))
            elif action_STR == "down":
                Current_Node.transfer_node_down = Search_Node(state=Output_state, parent=Current_Node, movement=action_STR, depth=depth, \
                                                      step_cost=Node_VALUE, path_cost=path_cost, heuristic_cost=h_cost)
                unvisited_q.append((Current_Node.transfer_node_down, total_cost))
                Stack_Depth.append((depth, total_cost))
                Stack_Path_cost.append((path_cost, total_cost))
        
        return Current_Node, unvisited_q, Stack_Depth, Stack_Path_cost 

    # once the goal node is found, trace back to the root node and print out the path
    def Ouput_PATH(self):
        # creating stacks for placing the trace
        STATE_Trace = [self.state]
        MOVES_trace = [self.movement]
        DEPTH_trace = [self.depth]
        Step_Trace = [self.step_cost]
        Path_Trace = [self.path_cost]
        Heuristic_Trace = [self.heuristic_cost]

        # add node information as tracing back up the tree
        while self.parent:
            self = self.parent
            STATE_Trace.append(self.state)
            MOVES_trace.append(self.movement)
            DEPTH_trace.append(self.depth)
            Step_Trace.append(self.step_cost)
            Path_Trace.append(self.path_cost)
            Heuristic_Trace.append(self.heuristic_cost)

        # print out the path
        step_Count = 0
        while STATE_Trace:
            print('step', step_Count)
            print(STATE_Trace.pop())
            print('movement=', MOVES_trace.pop(), ', depth=', str(DEPTH_trace.pop()))

            step_Count += 1
        print("=====Total Number of Moves in the Searching is =====: ",step_Count-1) # outside while loop, counter incremented by one

    def Depth_first_search(self, GOAL_state):

        UNVISITED_Stack = [self]  # Stack of found but unvisited nodes
        Stack_max_length = 1  

        Stack_Depth = [0]  # Stack of node depth
        Stack_Path_cost = [0]  # Stack for path cost
        Visited_Stack_Set = set([])  # record Visited_Stack_Set states

        while UNVISITED_Stack:

            # update maximum length of the Stack
            if len(UNVISITED_Stack) > Stack_max_length:
                Stack_max_length = len(UNVISITED_Stack)

            Current_Node = UNVISITED_Stack.pop()  # select and remove the first node in the Stack
            Current_Depth = Current_Node.depth
            
            # Current_Depth = Stack_Depth.pop()  # select and remove the depth for current node from the Stack
            
            if Current_Depth <= 10:             # Condition for Maximum Depth Exceeded.
                Current_path_COST = Stack_Path_cost.pop()  # select and remove the path cost for reaching current node
                Visited_Stack_Set.add(
                    tuple(Current_Node.state.reshape(1, 9)[0]))  # Remove repeated state, which is represented as a tuple
            # when the goal state is found, trace back to the root node and print out the path
                if np.array_equal(Current_Node.state, GOAL_state):
                    Current_Node.Ouput_PATH()
                    print("===== Number of States Enqueued in the DFS Searching is =====: ", len(UNVISITED_Stack))
                    return True

                else:
                    # see if moving left tile to the right is a valid move
                    if Current_Node.Transfer_RIGHT():
                        # Output_state, left_Node_Value = Current_Node.Transfer_RIGHT()
                        Output_state, Node_VALUE = Current_Node.Transfer_RIGHT()
                        Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost = self.Update_Nodes(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE,'right',Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost )


                    # see if moving upper tile down is a valid move
                    if Current_Node.Transfer_DOWN():
                        # Output_state, upper_Node_Value = Current_Node.Transfer_DOWN()
                        Output_state, Node_VALUE = Current_Node.Transfer_DOWN()
                        Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost = self.Update_Nodes(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE,'down',Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost )
                        

                    # see if moving lower tile up is a valid move
                    if Current_Node.Transfer_UP():
                        # Output_state, lower_Node_Value = Current_Node.Transfer_UP()
                        Output_state, Node_VALUE = Current_Node.Transfer_UP()
                        
                        Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost = self.Update_Nodes(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE,'up',Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost )
                
                    

                    # see if moving right tile to the left is a valid move
                    if Current_Node.Transfer_LEFT():
                        # Output_state, right_Node_Value = Current_Node.Transfer_LEFT()
                        Output_state, Node_VALUE = Current_Node.Transfer_LEFT()
                        Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost = self.Update_Nodes(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE,'left',Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost)
                
                    # check if the resulting node is already Visited_Stack_Set
                    
        
        print("Maximum Depth exceeded in your Search")

        return False


    def Iterative_Deepening_Search(self, GOAL_state):

        Stack_max_length = 1  # max number of nodes in the Stack, measuring space performance

        # search the tree till 10 levels in depth
        for depth_limit in range(11):
            print('depth limit',depth_limit)

            UNVISITED_Stack = [self]  # Stack of found but unvisited nodes
            Stack_Depth = [0]  # Stack of node depth
            Stack_Path_cost = [0]  # Stack for path cost
            Visited_Stack_Set = set([])  # Visited_Stack_Set states

            while UNVISITED_Stack:
                # update maximum length of the Stack
                if len(UNVISITED_Stack) > Stack_max_length:
                    Stack_max_length = len(UNVISITED_Stack)

                Current_Node = UNVISITED_Stack.pop()  # select and remove the first node in the Stack

                Current_Depth = Stack_Depth.pop()  # select and remove the depth for current node
                Current_path_COST = Stack_Path_cost.pop()  # select and avoid the path cost for reaching current node
                Visited_Stack_Set.add(tuple(Current_Node.state.reshape(1, 9)[0]))  # add state, which is represented as a tuple


                # when the goal state is found, trace back to the root node and print out the path
                if np.array_equal(Current_Node.state, GOAL_state):
                    Current_Node.Ouput_PATH()
                    print("===== Number of States Enqueued in the IDS Searching is =====: ", len(UNVISITED_Stack))
                    return True

                else:
                    if Current_Depth < depth_limit:

                        # see if moving upper tile down is a valid move
                        if Current_Node.Transfer_DOWN():
                            Output_state, Node_VALUE = Current_Node.Transfer_DOWN()
                            Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost = self.Update_Nodes(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE,'down',Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost)
                            # Output_state, upper_Node_Value = Current_Node.Transfer_DOWN()
                        
                            

                        # see if moving left tile to the right is a valid move
                        if Current_Node.Transfer_RIGHT():
                            Output_state, Node_VALUE = Current_Node.Transfer_RIGHT()
                            Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost = self.Update_Nodes(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE,'right',Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost)
                        

                        # see if moving lower tile up is a valid move
                        if Current_Node.Transfer_UP():
                            Output_state, Node_VALUE = Current_Node.Transfer_UP()
                            Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost = self.Update_Nodes(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE,'up',Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost)
                        
                            

                        # see if moving right tile to the left is a valid move
                        if Current_Node.Transfer_LEFT():
                            Output_state, Node_VALUE = Current_Node.Transfer_LEFT()
                            Current_Node, UNVISITED_Stack, Stack_Depth, Stack_Path_cost = self.Update_Nodes(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE,'left',Current_Depth,Current_path_COST,UNVISITED_Stack,Stack_Depth,Stack_Path_cost)
                        
                            
            if depth_limit>=10:
                print("Maximum Depth exceeded in your Search")
                

    # search based on path cost + heuristic cost
    def A_Star_Heuristic_search(self, GOAL_state, heuristic_function):

        unvisited_q = [
            (self, 0)]  # Stack of (found but unvisited nodes, path cost+heuristic cost), ordered by the second element
        Stack_max_length = 1  # max number of nodes in the Stack, measuring space performance

        Stack_Depth = [(0, 0)]  # Stack of node depth, (depth, path_cost+heuristic cost)
        Stack_Path_cost = [(0, 0)]  # Stack for path cost, (path_cost, path_cost+heuristic cost)
        Visited_Stack_Set = set([])  # record Visited_Stack_Set states

        while unvisited_q:
            # sort Stack based on path_cost+heuristic cost, in ascending order
            unvisited_q = sorted(unvisited_q, key=lambda x: x[1])
            Stack_Depth = sorted(Stack_Depth, key=lambda x: x[1])
            Stack_Path_cost = sorted(Stack_Path_cost, key=lambda x: x[1])

            # update maximum length of the Stack
            if len(unvisited_q) > Stack_max_length:
                Stack_max_length = len(unvisited_q)

            Current_Node = unvisited_q.pop(0)[0]  # select and remove the first node in the Stack
            Current_Depth = Stack_Depth.pop(0)[0]  # select and remove the depth for current node
            if Current_Depth<=10:
                Current_path_COST = Stack_Path_cost.pop(0)[0]  # # select and remove the path cost for reaching current node
                Visited_Stack_Set.add(
                    tuple(Current_Node.state.reshape(1, 9)[0]))  # avoid repeated state, which is represented as a tuple

            # when the goal state is found, trace back to the root node and print out the path
                if np.array_equal(Current_Node.state, GOAL_state):
                    Current_Node.Ouput_PATH()
                    print("===== Number of States Enqueued in the A Star(*) Searching is =====: ",len(unvisited_q))
                    return True

                else:
                # see if moving upper tile down is a valid move
                    if Current_Node.Transfer_DOWN():
                        Output_state, Node_VALUE = Current_Node.Transfer_DOWN()
                        Current_Node, unvisited_q, Stack_Depth, Stack_Path_cost = self.Update_Nodes_AStar_Search(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE, 'down',Current_Depth,Current_path_COST,unvisited_q,Stack_Depth,Stack_Path_cost, GOAL_state, heuristic_function)
                   

                # see if moving left tile to the right is a valid move
                    if Current_Node.Transfer_RIGHT():
                        Output_state, Node_VALUE = Current_Node.Transfer_RIGHT()
                        Current_Node, unvisited_q, Stack_Depth, Stack_Path_cost = self.Update_Nodes_AStar_Search(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE, 'right',Current_Depth,Current_path_COST,unvisited_q,Stack_Depth,Stack_Path_cost, GOAL_state, heuristic_function)
                  

                # see if moving lower tile up is a valid move
                    if Current_Node.Transfer_UP():
                        Output_state, Node_VALUE = Current_Node.Transfer_UP()
                        Current_Node, unvisited_q, Stack_Depth, Stack_Path_cost = self.Update_Nodes_AStar_Search(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE, 'up',Current_Depth,Current_path_COST,unvisited_q,Stack_Depth,Stack_Path_cost, GOAL_state, heuristic_function)
                    

                # see if moving right tile to the left is a valid move
                    if Current_Node.Transfer_LEFT():
                        Output_state, Node_VALUE = Current_Node.Transfer_LEFT()
                        Current_Node, unvisited_q, Stack_Depth, Stack_Path_cost = self.Update_Nodes_AStar_Search(Current_Node, Output_state,Visited_Stack_Set,Node_VALUE, 'left',Current_Depth,Current_path_COST,unvisited_q,Stack_Depth,Stack_Path_cost, GOAL_state, heuristic_function)
                

            else:
                print("Maximum Depth exceeded in your Search")
                return False
