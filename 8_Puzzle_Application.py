# <!-- // Code Author:-
# // Name: Shivam Gupta
# // Net ID: SXG190040
# // Artificial Intelligence (CS 6364.001) Assignment 1 (Search Algorithms) -->

import sys
import numpy as np
import Search_Algorithms

algorithm_name = sys.argv[1]
def Run_Search_Algorithms(algorithm_name, input_file_path):

    if input_file_path != "input_state.txt":
        print("====  Please Enter a Valid input filepath which is input_state.txt====")
        exit()

    print("Fetching the Input state from the input text file ")
    # input_file_path = r"input_state.txt"
    states = open(input_file_path, "r")
    for values in ( raw_data.strip().split() for raw_data in states):
        Initial_State = values

    Initial_State = list(map(int, Initial_State))
    Initial_State = np.array(Initial_State).reshape(3,3)
    Final_GOAL_State = np.array([1,2,3,4,5,6,7,8,0]).reshape(3,3)
    print("==============================")
    print("Initial Input State")
    print(Initial_State)
    print("==============================")
    print("Final GOAL Sate")
    print(Final_GOAL_State)

    Start_Root_Node = Search_Algorithms.Search_Node(state=Initial_State,parent=None,movement=None,depth=0,step_cost=0,path_cost=0,heuristic_cost=0)
    

    if algorithm_name=="dfs":
        print("====  Running Depth First Search Algorithm ====")
        Start_Root_Node.Depth_first_search(Final_GOAL_State)
    elif algorithm_name == "ids":
        print("====  Running Iterative Deepening Search Algorithm ====")
        Start_Root_Node.Iterative_Deepening_Search(Final_GOAL_State)
    elif algorithm_name== "astar1":
        print("====  Running A star(*) Search Algorithm using Manhattan Heuristic Function ====")
        Start_Root_Node.A_Star_Heuristic_search(Final_GOAL_State, heuristic_function = 'manhattan')
    elif algorithm_name== "astar2":
        print("====  Running A star(*) Search Algorithm using Number of Misplaced Heuristic Function ====")
        Start_Root_Node.A_Star_Heuristic_search(Final_GOAL_State, heuristic_function = 'num_misplaced')
    else:
        print("====  Please Enter a Valid Search Algorithm  ====")

if __name__ == "__main__":
    algorithm_name = sys.argv[1]
    input_file_path = sys.argv[2]
    Run_Search_Algorithms(algorithm_name, input_file_path)
