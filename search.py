# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
#     """
#     Search the deepest nodes in the search tree first.

#     Your search algorithm needs to return a list of actions that reaches the
#     goal. Make sure to implement a graph search algorithm.

#     To get started, you might want to try some of these simple commands to
#     understand the search problem that is being passed in:

#     print("Start:", problem.getStartState())
#     print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
#     print("Start's successors:", problem.getSuccessors(problem.getStartState()))
#     """
#     "*** YOUR CODE HERE ***"
#     print("Start:", problem.getStartState())
    start= problem.getStartState() # <----- our root / initial state
#     print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
#     print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    closed=[] # <----- an empyt set
    fringe =util.Stack() # <----- the fringe should be a stack because DFS follows LIFO (Last In First Out)
    fringe.push(start) # <------- adding our initial state into the fringe
    solution=[]  # <--------- our solution list that will return 
    totalPath=util.Stack() #<------ we will fill totalPath stack with our paths and the 'last in' would be our goal
    while True: # <---- loop do (such as stated in our project1 file)
        if fringe.isEmpty(): # <---- if fring is empyty then return faliure
            return -1 # <----- '-1' is faliure
        node=fringe.pop() # <-----we iniliaze our node by poping the fringe
        if problem.isGoalState(node)==True: # <--- test if our node is the goal state
            break # <------in case our node is the goal state we will break from the loop
        if node not in closed: # <---- checkif the node is or is not in our closed set
            closed.append(node)# <----- we add our node to the closed set
            successors = problem.getSuccessors(node) # <---- we get our node' successors 
#             print(closed) # <----To follow the nodes expanded 
            for child_node, direction, cost in successors: # <---- we go through each of the node' successors
                fringe.push(child_node)# <---- inserting the child node in the fringe 
                path = solution + [direction] # <----- saving the path from the parent mode -> child node
                totalPath.push(path) # <---- gathering all of the paths from parent node to child node
        solution = totalPath.pop() # <---- poping the last path which will be our goal path
    return solution  # returning the soloution path
    
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start= problem.getStartState() # <----- our root / initial state
#     print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
#     print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    closed=[] # <----- an empyt set
    fringe =util.Queue() # <----- the fringe should be a stack because BFS follows FIFO (First In First Out)
    fringe.push(start) # <------- adding our initial state into the fringe
    solution=[]  # <--------- our solution list that will return 
    totalPath=util.Queue() #<------ we will fill totalPath stack with our paths and the 'First in' would be our goal
    while True: # <---- loop do (such as stated in our project1 file)
        if fringe.isEmpty(): # <---- if fring is empyty then return faliure
            return -1 # <----- '-1' is faliure
        node=fringe.pop() # <-----we iniliaze our node by poping the fringe
        if problem.isGoalState(node)==True: # <--- test if our node is the goal state
            break # <------in case our node is the goal state we will break from the loop
        if node not in closed: # <---- checkif the node is or is not in our closed set
            closed.append(node)# <----- we add our node to the closed set
            successors = problem.getSuccessors(node) # <---- we get our node' successors 
#             print(closed) # <----To follow the nodes expanded 
            for child_node, direction, cost in successors: # <---- we go through each of the node' successors
                fringe.push(child_node)# <---- inserting the child node in the fringe 
                path = solution + [direction] # <----- saving the path from the parent mode -> child node
                totalPath.push(path) # <---- gathering all of the paths from parent node to child node
        solution = totalPath.pop() # <---- poping the last path which will be our goal path
    return solution  # returning the soloution path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start= problem.getStartState() # <----- our root / initial state
#     print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
#     print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    closed=[] # <----- an empyt set
    fringe =util.PriorityQueue() # <----- the fringe should be a PriorityQueue because of using UCS
    InitialCost=0 # <----- our cost in the inital state
    fringe.push(start,InitialCost) # <------- adding our initial state into the fringe
    solution=[]  # <--------- our solution list that will return 
    totalPath=util.PriorityQueue() #<------ we will fill totalPath PriorityQueue with our paths and the 'least cose' would be our goal
    while True: # <---- loop do (such as stated in our project1 file)
        if fringe.isEmpty(): # <---- if fring is empyty then return faliure
            return -1 # <----- '-1' is faliure
        node=fringe.pop() # <-----we iniliaze our node by poping the fringe
        if problem.isGoalState(node)==True: # <--- test if our node is the goal state
            break  # <------in case our node is the goal state we will break from the loop
        if node not in closed: # <---- checkif the node is or is not in our closed set
            closed.append(node)# <----- we add our node to the closed set
            successors = problem.getSuccessors(node) # <---- we get our node' cost 
#             print(closed) #To follow the nodes expanded
            for child_node, direction, cost in successors:# <---- we go through each of the node' successors
                path = solution + [direction] #storing the path using the direction of the child node
                if child_node not in closed:  
                    actionCOST = problem.getCostOfActions(path) # computing the cost of each path found
                    fringe.push(child_node,actionCOST)# <---- inserting the child node & cost in the fringe
                    totalPath.push(path, actionCOST)  # <---- gathering all of the paths from parent node to child node
                    
        solution = totalPath.pop() # <---- poping the last path which will be our goal path
    return solution  # returning the soloution path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start= problem.getStartState() # <----- our root / initial state
#     print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
#     print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    closed=[] # <----- an empyt set
    fringe =util.PriorityQueue() # <----- the fringe should be a PriorityQueue because of using A*
    InitialCost=0 # <----- our cost in the inital state
    fringe.push(start,InitialCost) # <------- adding our initial state into the fringe
    solution=[]  # <--------- our solution list that will return 
    totalPath=util.PriorityQueue() #<------ we will fill totalPath PriorityQueue with our paths and the 'least cose + heuristic' would be our goal
    while True: # <---- loop do (such as stated in our project1 file)
        if fringe.isEmpty(): # <---- if fring is empyty then return faliure
            return -1 # <----- '-1' is faliure
        node=fringe.pop() # <-----we iniliaze our node by poping the fringe
        if problem.isGoalState(node)==True: # <--- test if our node is the goal state
            break  # <------in case our node is the goal state we will break from the loop
        if node not in closed: # <---- checkif the node is or is not in our closed set
            closed.append(node)# <----- we add our node to the closed set
            successors = problem.getSuccessors(node) # <---- we get our node' cost 
#             print(closed) #To follow the nodes expanded
            for child_node, direction, cost in successors:# <---- we go through each of the node' successors
                path = solution + [direction]#storing the path using the direction of the child node
                if child_node not in closed:  
                    actionCOST = problem.getCostOfActions(path) + heuristic(child_node,problem) # computing the cost of each path found + heuristic
                    fringe.push(child_node,actionCOST)# <---- inserting the child node & cost in the fringe
                    totalPath.push(path, actionCOST)  # <---- gathering all of the paths from parent node to child node
                    
        solution = totalPath.pop() # <---- poping the last path which will be our goal path
    return solution  # returning the soloution path
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
