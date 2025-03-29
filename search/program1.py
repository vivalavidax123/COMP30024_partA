# COMP30024 Artificial Intelligence, Semester 1 2025
# Project Part A: Single Player Freckers


from .core import CellState, Coord, Direction, MoveAction, BOARD_N
from .utils import render_board
import heapq


def a_star_search(board: dict[Coord, CellState]) -> list[MoveAction] | None:
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

    # Find the initial position of the Red frog
    red_frog = None
    for coord, state in board.items():
        if state == CellState.RED:
            red_frog = coord
            break
    
    # Priority queue for A* search and a set for visited positions
    # Each element is (priority, unique_id, (position, actions))
    # unique_id is used to break ties when priorities are equal
    priority_queue = []
    counter = 0  # For generating unique IDs
    
    # Start with the red frog's position, no actions, and priority based on heuristic
    initial_priority = heuristic(red_frog)
    heapq.heappush(priority_queue, (initial_priority, counter, (red_frog, [])))
    
    visited = set()
    visited.add(red_frog)  # Mark the starting position as visited
    
    while priority_queue:
        # Get the position with the lowest f(n) = g(n) + h(n)
        _, _, (current_pos, actions) = heapq.heappop(priority_queue)
        
        # Check if we've reached the goal (row 7)
        if current_pos.r == 7:
            return actions
        
        # Try all possible moves from current position
        possible_moves = get_possible_moves(current_pos, board)
        
        for move, new_pos in possible_moves:
            if new_pos not in visited:
                visited.add(new_pos)
                
                # Calculate priority for A*
                # g(n) = cost so far (number of moves)
                g_n = len(actions) + 1
                # h(n) = heuristic estimate to goal
                h_n = heuristic(new_pos)
                # f(n) = g(n) + h(n)
                f_n = g_n + h_n
                
                # Increment counter for unique ID
                counter += 1
                
                # Add to priority queue
                heapq.heappush(priority_queue, (f_n, counter, (new_pos, actions + [move])))
    
    return None  # No solution found
   

def heuristic(coord: Coord) -> int:
    # Heuristic function: Manhattan distance to the goal row (7)
    # The goal is to reach row 7, so we want to minimize the distance to it
    return 7 - coord.r


# Check if a coordinate is valid (within the board boundaries)
def is_valid_move(r, c):
    return 0 <= r < BOARD_N and 0 <= c < BOARD_N


# Check if a direction is orthogonal
def is_orthogonal(direction):
    return (direction == Direction.Down or 
            direction == Direction.Left or 
            direction == Direction.Right)


# Get all possible moves from a given position
def get_possible_moves(pos: Coord, board: dict[Coord, CellState]) -> list[tuple[MoveAction, Coord]]:
    moves = []
    
    # For each direction, check if we can move or jump
    for direction in Direction:
        # Calculate new position without using the Coord addition (which wraps)
        new_r = pos.r + direction.r
        new_c = pos.c + direction.c
        
        # Skip invalid moves (out of bounds)
        if not is_valid_move(new_r, new_c):
            continue
            
        # Create the adjacent position
        adjacent_pos = Coord(new_r, new_c)
        
        # Check for regular moves to lily pads
        if board.get(adjacent_pos) == CellState.LILY_PAD and is_orthogonal(direction):
            moves.append((MoveAction(pos, direction), adjacent_pos))
            
        # Check for jumps over blue frogs
        jumps = find_valid_jumps(pos, board)
        for direction, landing_pos in jumps:
            moves.append((MoveAction(pos, direction), landing_pos))
            
            # Check for multi-jumps
            jump_visited = set([pos, landing_pos])
            check_multi_jumps(pos, landing_pos, [direction], board, moves, jump_visited)
            
    return moves


# Check for multi-jumps recursively
# This function explores all possible multi-jumps from the current position
def check_multi_jumps(orig_pos, curr_pos, dirs_so_far, board, moves, jump_visited):
    if len(dirs_so_far) > 10:
        return
        
    # Find further jumps from this position
    jumps = find_valid_jumps(curr_pos, board, jump_visited)
    
    for direction, landing_pos in jumps:
        updated_dirs = dirs_so_far + [direction]
        moves.append((MoveAction(orig_pos, updated_dirs), landing_pos))
        
        # Update visited and continue exploring
        jump_visited.add(landing_pos)
        check_multi_jumps(orig_pos, landing_pos, updated_dirs, board, moves, jump_visited)


# Find all valid jumps from the current position over blue frogs
def find_valid_jumps(pos, board, jump_visited=None):
    jumps = []
    
    for direction in Direction:
        # Calculate adjacent position
        new_r = pos.r + direction.r
        new_c = pos.c + direction.c
        
        if not is_valid_move(new_r, new_c):
            continue
            
        adjacent_pos = Coord(new_r, new_c)
        
        # Check if there's a blue frog to jump over
        if board.get(adjacent_pos) == CellState.BLUE:
            # Calculate landing position
            landing_r = new_r + direction.r
            landing_c = new_c + direction.c
            
            if not is_valid_move(landing_r, landing_c):
                continue
                
            landing_pos = Coord(landing_r, landing_c)
            
            # Check if landing is valid (must be a lily pad) and not already visited
            if (board.get(landing_pos) == CellState.LILY_PAD and 
                (jump_visited is None or landing_pos not in jump_visited)):
                jumps.append((direction, landing_pos))
            
    return jumps