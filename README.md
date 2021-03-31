# <!-- // Code Author:-
# // Name: Shivam Gupta
# // Net ID: SXG190040
# // Artificial Intelligence (CS 6364.001) Assignment 1 (Search Algorithms) -->

## Implementation of Search Algorithms


## Compiling and Running the Scripts:
Before running keep all the files ```Homework1.py```, ```Search_Algorithms.py```, ```input_state.txt```
in the same folder to run the script

## Search Algorithms: How to run the Script
```python Homework1.py <algorithm_name> <input_file_path>```


## How to Use the Scripts:
# For Running Depth First Search Algorithm 
    <algorithm_name> == "dfs"
    For example Run command:-:
```python Homework1.py dfs input_state.txt```
on UTD CS Linux Servers / Anaconda Prompt/Command Prompt
    
# For Running Iterative Deepening Search Algorithm 
    <algorithm_name> == "ids"
    For example Run command:-:  
```python Homework1.py ids input_state.txt```
on UTD CS Linux Servers / Anaconda Prompt/Command Prompt
    
# For Running A star(*) Search Algorithm using Manhattan Heuristic Function
    <algorithm_name> == "astar1"
    For example Run command:-:
```python Homework1.py astar1 input_state.txt```
on UTD CS Linux Servers / Anaconda Prompt/Command Prompt
    
# For Running A star(*) Search Algorithm using Number of Misplaced Heuristic Function 
    <algorithm_name> == "astar2"
    For example Run command:-:
```python Homework1.py astar2 input_state.txt```
on UTD CS Linux Servers / Anaconda Prompt/Command Prompt



# Note: 0 is my empty tile in the given input stae and the Goal state as well



## Results
The Results Can be seen on the Output Console after running the Python File and is shown
over here with sample input state

Input file is ```input_state.txt```
You can give any input state in the given text file. For example curretly it contains the valuelike this, you can change as per the requuirement:
0 1 3 4 2 5 7 8 6
### Note:  Where 0 represents my Empty Tile instead of *

# Example: 
Input State: 
array([[0, 1, 3],
       [4, 2, 5],
       [7, 8, 6]])   Where 0 is my Empty Tile instead of *

# Outputs from the CS Grad Server Console:

## For Running Depth First Search Algorithm 
Fetching the Input state from the input text file
==============================
Initial Input State
[[0 1 3]
 [4 2 5]
 [7 8 6]]
==============================
Final GOAL Sate
[[1 2 3]
 [4 5 6]
 [7 8 0]]
====  Running Depth First Search Algorithm ====
step 0
[[0 1 3]
 [4 2 5]
 [7 8 6]]
movement= None , depth= 0
step 1
[[1 0 3]
 [4 2 5]
 [7 8 6]]
movement= left , depth= 1
step 2
[[1 2 3]
 [4 0 5]
 [7 8 6]]
movement= up , depth= 2
step 3
[[1 2 3]
 [4 5 0]
 [7 8 6]]
movement= left , depth= 3
step 4
[[1 2 3]
 [4 5 6]
 [7 8 0]]
movement= up , depth= 4
=====Total Number of Moves in the Searching is =====:  4
===== Number of States Enqueued in the DFS Searching is =====:  3

## For Running Iterative Deepening Search Algorithm 
Fetching the Input state from the input text file
==============================
Initial Input State
[[0 1 3]
 [4 2 5]
 [7 8 6]]
==============================
Final GOAL Sate
[[1 2 3]
 [4 5 6]
 [7 8 0]]
====  Running Iterative Deepening Search Algorithm ====
depth limit 0
depth limit 1
depth limit 2
depth limit 3
depth limit 4
step 0
[[0 1 3]
 [4 2 5]
 [7 8 6]]
movement= None , depth= 0
step 1
[[1 0 3]
 [4 2 5]
 [7 8 6]]
movement= left , depth= 1
step 2
[[1 2 3]
 [4 0 5]
 [7 8 6]]
movement= up , depth= 2
step 3
[[1 2 3]
 [4 5 0]
 [7 8 6]]
movement= left , depth= 3
step 4
[[1 2 3]
 [4 5 6]
 [7 8 0]]
movement= up , depth= 4
=====Total Number of Moves in the Searching is =====:  4
===== Number of States Enqueued in the IDS Searching is =====:  4

## For Running A star(*) Search Algorithm using Manhattan Heuristic Function
Fetching the Input state from the input text file
==============================
Initial Input State
[[0 1 3]
 [4 2 5]
 [7 8 6]]
==============================
Final GOAL Sate
[[1 2 3]
 [4 5 6]
 [7 8 0]]
====  Running A star(*) Search Algorithm using Manhattan Heuristic Function ====
step 0
[[0 1 3]
 [4 2 5]
 [7 8 6]]
movement= None , depth= 0
step 1
[[1 0 3]
 [4 2 5]
 [7 8 6]]
movement= left , depth= 1
step 2
[[1 2 3]
 [4 0 5]
 [7 8 6]]
movement= up , depth= 2
step 3
[[1 2 3]
 [4 5 0]
 [7 8 6]]
movement= left , depth= 3
step 4
[[1 2 3]
 [4 5 6]
 [7 8 0]]
movement= up , depth= 4
=====Total Number of Moves in the Searching is =====:  4
===== Number of States Enqueued in the A Star(*) Searching is =====:  11


## For Running A star(*) Search Algorithm using Number of Misplaced Heuristic Function 
Fetching the Input state from the input text file
==============================
Initial Input State
[[0 1 3]
 [4 2 5]
 [7 8 6]]
==============================
Final GOAL Sate
[[1 2 3]
 [4 5 6]
 [7 8 0]]
====  Running A star(*) Search Algorithm using Number of Misplaced Heuristic Function ====
step 0
[[0 1 3]
 [4 2 5]
 [7 8 6]]
movement= None , depth= 0
step 1
[[1 0 3]
 [4 2 5]
 [7 8 6]]
movement= left , depth= 1
step 2
[[1 2 3]
 [4 0 5]
 [7 8 6]]
movement= up , depth= 2
step 3
[[1 2 3]
 [4 5 0]
 [7 8 6]]
movement= left , depth= 3
step 4
[[1 2 3]
 [4 5 6]
 [7 8 0]]
movement= up , depth= 4
=====Total Number of Moves in the Searching is =====:  4
===== Number of States Enqueued in the A Star(*) Searching is =====:  12



# A Short comparative analysis of two heuristics used for A*

## Manhattan distance heuristic:
* Manhattan distance heuristic function is basically the sum of the differences between the x and the y coordinates of both those points.
* Mathematical Formula in my code:
Goal_Mapping = {1: (0, 0), 2: (0, 1), 3: (0, 2), 4: (1, 0), 5: (1, 1), 
                6: (1, 2), 7: (2, 0), 8: (2, 1),0: (2, 2)}
Mahatten_Cost += sum(abs(p - q) for p, q in zip((i, j), Goal_Mapping[current_state[i, j]]))
* In my given Initial Input State in the ```input_state.txt``` file
                [[0 1 3]
                [4 2 5]
                [7 8 6]]
* Total number of Movements for searching from input state to final state = 4
* Searching went till depth 4
* Total number of states enqueued in my stack was =  11


## Number of Misplaced Tile heuristic:
* Misplaced Tile heuristic is basically the number of misplaced tiles by comparing the initial state of the 8-puzzle Matrix and how many tiles apart that tile is from the Final goal state 8-Puzzle Matrix.
* Mathematical Formula in my code:
misplaced_cost = np.sum(current_state != GOAL_state) - 1  
* In my given Initial Input State in the ```input_state.txt``` file
                [[0 1 3]
                [4 2 5]
                [7 8 6]]
* Total number of Movements for searching from input to final state = 4
* Searching went till depth 4
* Total number of states enqueued in my stack was =  12

### Conclusion on Comparative analysis of two heuristics used for A* Algorithms:
According to me Manhatten distance heuristic could be considered better than Number of Misplaced heuristic because Number of Misplaced heuristic considers only whether the tile is misplaced or not it doesn't inform you that how far it is from the final state where as Manhatten distance heuristic considers that information as well which helps in improving the searching. This conclusion is from my intuition and the experiment which I did on some of the input states by running my script on it!
