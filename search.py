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
import state as state

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getInitialState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isFinalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getNextStates(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getActionCost(self, actions):
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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    from util import Stack
    nextNode = Stack()
    visitedNodes = set()
    startState = problem.getInitialState()  # starting point (state)
    path = []  # Here the directions will be stored, initialized as blank

    if problem.isFinalState(startState):  # Checks if starting position is winning position
        return path

    node = (startState, path)
    nextNode.enqueue(node)

    while not nextNode.isEmpty():
        state, path = nextNode.dequeue()

        if state not in visitedNodes:  # Will not run if node has already been visited
            visitedNodes.add(state)
            if problem.isFinalState(state):  # End condition, returns the directions
                return path
            for successor in problem.getNextStates(state):  # Gets the neighboring nodes
                node = (successor[0], path + [successor[1]])
                nextNode.enqueue(node)  # and adds them to the queue to be visited


def breadthFirstSearch(problem):
    from util import Queue

    # print("Start:", problem.getInitialState())
    # print("Is the start a goal?", problem.isFinalState(problem.getInitialState()))
    # print("Start's successors:", problem.getNextStates(problem.getInitialState()))
    nextNode = Queue()
    visitedNodes = set()
    startState = problem.getInitialState()  # starting point (state)
    path = []  # Here the directions will be stored, initialized as blank
    if problem.isFinalState(startState):  # Checks if starting position is winning position
        return path

    node = (startState, path)
    nextNode.enqueue(node)

    while not nextNode.isEmpty():
        state, path = nextNode.dequeue()

        if state not in visitedNodes:  # Will not run if node has already been visited
            visitedNodes.add(state)
            if problem.isFinalState(state):  # End condition, returns the directions
                return path
            for successor in problem.getNextStates(state):  # Gets the neighboring nodes
                node = (successor[0], path + [successor[1]])
                nextNode.enqueue(node)  # and adds them to the queue to be visited


def uniformCostSearch(problem):
    from util import PriorityQueue
    """Search the node of least total cost first."""

    startState=problem.getInitialState()
    if problem.isFinalState(startState):
        return []

    visitedNodes=set()
    nextNode=PriorityQueue()
    nextNode.enqueue((startState,[],0), 0)

    while not nextNode.isEmpty():
        node, path, cost =nextNode.dequeue()
        if node not in visitedNodes:
            visitedNodes.add(node)
            if problem.isFinalState(node):
                return path
            for next, action, newCost in problem.getNextStates(node):
                action=path+[action]
                newCost= cost +newCost
                nextNode.enqueue((next, action, newCost),newCost)



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
   from util import PriorityQueue

   startState=problem.getInitialState()
   if problem.isFinalState(startState):
       return []

   visitedNodes=set()
   nextNode=PriorityQueue()
   nextNode.enqueue((startState,[],0), 0)

   while not nextNode.isEmpty():
       node, path, cost =nextNode.dequeue()
       if node not in visitedNodes:
           visitedNodes.add(node)
           if problem.isFinalState(node):
               return path
           for next, action, newCost in problem.getNextStates(node):
               action=path+[action]
               newCost= cost +newCost
               heuristicCost= newCost+ heuristic(next, problem)
               nextNode.enqueue((next, action, newCost),heuristicCost)



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
