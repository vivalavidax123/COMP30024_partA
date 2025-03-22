# COMP30024 Artificial Intelligence, Semester 1 2025
# Project Part A: Single Player Freckers
import heapq

from .core import CellState, Coord, Direction, MoveAction
from .utils import render_board

#import heapq

def search(
    board: dict[Coord, CellState]
) -> list[MoveAction] | None:
    """
    This is the entry point for your submission. You should modify this
    function to solve the search problem discussed in the Part A specification.
    See `core.py` for information on the types being used here.

    Parameters:
        `board`: a dictionary representing the initial board state, mapping
            coordinates to "player colours". The keys are `Coord` instances,
            and the values are `CellState` instances which can be one of
            `CellState.RED`, `CellState.BLUE`, or `CellState.LILY_PAD`.

    Returns:
        A list of "move actions" as MoveAction instances, or `None` if no
        solution is possible.
    """

    # The render_board() function is handy for debugging. It will print out a
    # board state in a human-readable format. If your terminal supports ANSI
    # codes, set the `ansi` flag to True to print a colour-coded version!
    print(render_board(board, ansi=True))
    #find the start and end coordinates
    #start
    start = None
    for coord in board:
        if board[coord] == CellState.RED:
            start = coord

    ends = []
    for i in range(8):
        candidate = Coord(7, i)
        if valid_landing_spot(board, candidate):
            ends.append(candidate)

    if ends is not None:
        #pathfinding
        path = pathfinding(board, start, ends)
        if path is not None:
            return path
        #[Coord, Direction]
    return None

#a* search
def pathfinding(board: dict[Coord, CellState], start:[Coord] , ends :[[Coord]]) \
        -> list[MoveAction] | None:
    action_list = [] # list of actions and sorted by f value
    closed_list = set()
    start_node = Node(start)
    #also filter out the ends that are not valid landing spots
    end_nodes = [Node(end) for end in ends if valid_landing_spot(board, end)]

    #the current node
    action_list.append(start_node)

    #continue exploring until open list is empty
    while len(action_list) > 0:
        current = action_list.pop(0)#get the node with smallest f value
        print("debug node" + str(current.coord) + " " + str([current.moves]))

        closed_list.add(current.coord)

        #check if current is the end
        if current.coord in ends:
            #repack the path to return
            return [MoveAction(coord=o.coord, _directions= o.moves)
                    for o in action_list]

        #add adjacent coordinates to open list
        pending_directions = [MoveAction]
        # get surrounding coordinates
        for direction in red_directions():
            next_coord = current.coord + direction
            #if it can jump
            new_node = None
            if next_coord not in closed_list:
                #add the next node to the open list
                if can_jump(board, next_coord, direction):
                    new_node = Node(next_coord + direction)
                #if it cannot jump
                elif valid_landing_spot(board, next_coord):
                    new_node = Node(next_coord)
                else:
                    continue
                new_node.g = current.g + 1
                new_node.h = g_cost(new_node.coord, end_nodes)
                new_node.f = new_node.g + new_node.h
                new_node.moves.append(direction)
                action_list.append(new_node)
                action_list.sort(key=lambda x: x.f)
def red_directions():
    return [Direction.Down, Direction.DownLeft, Direction.DownRight, Direction.Left, Direction.Right]

def can_jump(board, new_coord, direction):
    #check if there is a frog
    if new_coord not in board:
        return False
    if board[new_coord] == CellState.BLUE:
        if (new_coord + direction) in board and board[new_coord + direction] == CellState.LILY_PAD:
            return True
    return False
#heuristic function
def g_cost(start : Coord, ends : []) -> int:
    return min(abs(start.r - end.coord.r) + abs(start.c - end.coord.c) for end in ends)

def valid_landing_spot(board, coord):
    #check position is valid
    if coord not in board:
        return False
    #check valid pad
    if board[coord] == CellState.LILY_PAD and board[coord] != CellState.RED and board[coord] != CellState.BLUE:
        return True

class Node:
    def __init__(self, coord):
        self.coord = coord
        self.g = 0
        self.h = 0
        self.f = 0
        self.moves = []
    def __eq__(self, other):
        return self.coord == other.coord