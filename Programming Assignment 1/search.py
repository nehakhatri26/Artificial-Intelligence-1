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
import heapq

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
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    
    "Defining a stack to store the values of fringe nodes"
    fringe_stack = util.Stack()  
    "Defining a set to store all the visited nodes"
    traversed_nodes = set()
    "getting the start state of the node"
    initial_Node = (problem.getStartState(),[])
    "push the initial state to the fringe stack"
    fringe_stack.push(initial_Node)
    "adding the nodes to the visited nodes set"
    traversed_nodes.add( problem.getStartState())
    "check if the fringe stack is not empty"
    while fringe_stack.isEmpty() == 0:                    
        value = fringe_stack.pop()
        if problem.isGoalState(value[0]):
            break
        else:
            if value[0] not in traversed_nodes:
                traversed_nodes.add(value[0])
            "getting the successors for the start node"
            successor_node = problem.getSuccessors(value[0])
            successor_nodelist = list(successor_node)
            if(successor_nodelist > 0):
                for nodes in successor_nodelist:
                    if nodes[0] not in traversed_nodes:
                        "pushing the node and the path value to the stack"
                        fringe_stack.push( (nodes[0], value[1] + [nodes[1]]) )
                        "adding the visited nodes to the set"
                        traversed_nodes.add( nodes[0] )
    return value[1]

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
   
    "Defining a queue to store the values of fringe nodes"
    fringe_queue = util.Queue()
    "Defining a set to store all the visited nodes"
    traversed_nodes = set()
    plan = []
    "getting the start state of the node"
    initial_Node = (problem.getStartState(),[])
    "push the initial state to the fringe queue"
    fringe_queue.push(initial_Node)
    "check if the fringe queue is not empty"
    while fringe_queue.isEmpty() == 0:
        value = fringe_queue.pop()
        if problem.isGoalState(value[0]):
            break
        else:
            if value[0] not in traversed_nodes:
                traversed_nodes.add(value[0])
            "getting the successors for the start node"
            successor_node = problem.getSuccessors(value[0])
            successor_nodelist = list(successor_node)
            if(successor_nodelist > 0):
                "traversing through the queue"
                for nodes in successor_nodelist:
                    if nodes[0] not in traversed_nodes:
                        "pushing the node and the path value to the queue"
                        fringe_queue.push( (nodes[0], value[1] + [nodes[1]]) )
                        "adding the visited nodes to the set"
                        traversed_nodes.add( nodes[0] )
    return value[1]
                


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    "Defining a queue to store the values of fringe nodes"
    fringe_queue = util.PriorityQueue()
    "Defining a set to store all the visited nodes"
    traversed_nodes = set()
    "getting the start state of the node"
    initial_Node = (problem.getStartState(),[])
    "push the initial state to the fringe queue"
    fringe_queue.push(initial_Node,0)
    "check if the fringe queue is not empty"
    while fringe_queue.isEmpty() == 0:
        value = fringe_queue.pop()
        if problem.isGoalState(value[0]):
            break
        else:
            if value[0] not in traversed_nodes:
                traversed_nodes.add(value[0])
                "getting the successors for the start node"
                successor_node = problem.getSuccessors(value[0])
                successor_nodelist = list(successor_node)
                "traversing through the queue"
                if(successor_nodelist > 0):
                    for nodes in successor_nodelist:
                        if nodes[0] not in traversed_nodes:
                            "pushing all the children with the total cost as priority"
                            fringe_queue.push((nodes[0], value[1] + [nodes[1]]), problem.getCostOfActions(value[1] + [nodes[1]]))
                            "adding the visited nodes to the set"
            traversed_nodes.add(value[0])
    return value[1]
    util.raiseNotDefined()
    
        
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    "Defining a queue to store the values of fringe nodes"
    fringe_queue = util.PriorityQueue()
    "Defining a set to store all the visited nodes"
    traversed_nodes = set()
    "getting the start state of the node"
    initial_Node = (problem.getStartState(),[],0)
    "push the initial node to the fringe queue along with a null heuristic"
    fringe_queue.push((initial_Node), heuristic(problem.getStartState(), problem))
    "check if the fringe queue is not empty"
    while fringe_queue.isEmpty() == 0:
        value = fringe_queue.pop()
        if problem.isGoalState(value[0]):
            break
        else:
            "pushing the values on the basis of sum of absolute values of differences in the goal’s x and y coordinates and the current cell’s x and y coordinates respectively"
            if value[0] not in traversed_nodes:
                traversed_nodes.add(value[0])
                for nodes in problem.getSuccessors(value[0]):
                    fringe_queue.push((nodes[0],(value[1] + [nodes[1]]),(value[2]  + nodes[2])),((value[2]  + nodes[2]) + heuristic(nodes[0],problem))) 
        
    return value[1]
    util.raiseNotDefined()
    
       
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

"references:"
"https://www.geeksforgeeks.org/a-search-algorithm/"
"https://geekyisawesome.blogspot.com/2014/11/dijkstras-algorithm-and-search-graph.html"

