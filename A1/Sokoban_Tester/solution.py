#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import itertools
import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS, UP, DOWN, RIGHT, LEFT  # for Sokoban specific classes and problems

# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.
    
    trace = True
    
    if trace:
        print_state(state)
        
    obstacles = identify_obstacle(state)
    
    if has_box_at_corner(state.boxes, state.storage, obstacles):
        if trace:
            print("evalutaed to: ", math.inf, "by corner")
        return math.inf
    
    if has_unmovable_box(state, obstacles):
        if trace:
            print("evalutaed to: ", math.inf, "by unmovable box")
        return math.inf
    
    if edge_deadlock(state):
        if trace:
            print("evalutaed to: ", math.inf, "by edge deadlock")
        return math.inf
    
    if has_box_aggregate(state):
        if trace:
            print("evalutaed to: ", math.inf, "by box aggregate")
        return math.inf

    box_to_goal_min_dist, pairs = minimum_manhattan_pairs_distance(state, obstacles)
    
    if edge_unspecificity_deadlock(pairs, state):
        if trace:
            print("evalutaed to: ", math.inf, "by edge deadlock 2")
        return math.inf
    
    if has_unreachable_box(pairs, obstacles):
        if trace:
            print("evalutaed to: ", math.inf, "by unreachable box")
        return math.inf
    
    remaining_boxes = non_goal_box(state)
    robot_to_remaining_box_min_dist = sum(min(abs(bx - sx) + abs(by - sy) for (sx, sy) in remaining_boxes) for (bx, by) in state.robots) if remaining_boxes else 0
    # robot_to_goal_min_dist = sum(min(abs(bx - sx) + abs(by - sy) for (sx, sy) in state.storage) for (bx, by) in state.robots)

    dist = box_to_goal_min_dist + 0.05 * robot_to_remaining_box_min_dist
    
    
    if trace:
        print("evalutaed to: ", dist)
    
    return dist


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    return sum(
        min(abs(bx - sx) + abs(by - sy) for (sx, sy) in state.storage) for (bx, by) in state.boxes
    )

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval  # CHANGE THIS

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    
    se = SearchEngine('custom', 'default') # Note: Should it be something other than default?
    
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    result = se.search(timebound=timebound)
    
    return result 

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    
    stoptime = get_stoptime(timebound)
    
    se = SearchEngine('custom', 'default') # Note: Should it be something other than default?
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    
    curr_best = False
    result, stat = se.search(get_timebound(stoptime))
    while result:
        curr_best = result
        result, stat = se.search(get_timebound(stoptime), costbound=(math.inf, math.inf, curr_best.gval))
    
    return curr_best, stat
        
        
def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    
    stoptime = get_stoptime(timebound)
    
    se = SearchEngine('best_first', 'default') # Note: Should it be something other than default?
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    
    curr_best = False
    result, stat = se.search(get_timebound(stoptime))
    while result:
        curr_best = result
        result, stat = se.search(get_timebound(stoptime), costbound=(curr_best.gval, math.inf, math.inf))
    
    return curr_best, stat

def get_stoptime(timebound):
    start = os.times()[0]
    stop = None
    if timebound:
        stop = start + timebound
    return stop
    
def get_timebound(stoptime):
    starttime = os.times()[0]
    return stoptime - starttime
    

# Helpers
def identify_obstacle(state):
    obstacles = {coord for coord in state.obstacles}
    # for coord in state.boxes:
    #     obstacles.add(coord) 
    
    for i in range(-1, state.height):
        obstacles.add((-1, i))
        obstacles.add((state.width, i))
    for i in range(-1, state.width):
        obstacles.add((i, -1))
        obstacles.add((i, state.height))
    return obstacles

def has_box_at_corner(boxes, goals, obstacle):
    for box in boxes:
        if is_box_at_corner(box, goals, obstacle):
            return True 
    return False

def is_box_at_corner(box, goals, obstacle):
    u = UP.move(box) in obstacle
    r = RIGHT.move(box) in obstacle
    d = DOWN.move(box) in obstacle
    l = LEFT.move(box) in obstacle
    
    return box not in goals and ((u and r) or (r and d) or (d and l) or (l and u))


def edge_deadlock(state):
    
    northedge_storage = [(x,y) for (x, y) in state.storage if y == 0]
    northedge_box = [(x,y) for (x, y) in state.boxes if y == 0]
    if has_adjacent_box_not_at_goal(northedge_box, northedge_box, northedge_storage):
        return True
    
    southedge_storage = [(x,y) for (x, y) in state.storage if y == state.height - 1]
    southedge_box = [(x,y) for (x, y) in state.boxes if y == state.height - 1]
    if has_adjacent_box_not_at_goal(southedge_box, southedge_box, southedge_storage):
        return True
    
    westedge_storage = [(x,y) for (x, y) in state.storage if x == 0]
    westedge_box = [(x,y) for (x, y) in state.boxes if x == 0]
    if has_adjacent_box_not_at_goal(westedge_box, westedge_box, westedge_storage):
        return True
    
    eastedge_storage = [(x,y) for (x, y) in state.storage if x == state.width - 1]
    eastedge_box = [(x,y) for (x, y) in state.boxes if x == state.width - 1]
    if has_adjacent_box_not_at_goal(eastedge_box, eastedge_box, eastedge_storage):
        return True
    
    return False

def is_at_edge(obj, state):
    
    if obj[1] == 0:
        return 1
    elif obj[0] == state.width - 1:
        return 2
    elif obj[1] == state.height - 1:
        return 3
    elif obj[0] == 0:
        return 4
    else:
        return 0

def edge_unspecificity_deadlock(pairs, state):
    
    for goal, box in pairs:
        if box is None:
            continue
        if is_at_edge(goal, state) == 0 and is_at_edge(box, state) != 0:
            return True


def has_adjacent_box_not_at_goal(batch1, batch2, goals):
    for obj1 in batch1:
        for obj2 in batch2:
            if is_adjacent(obj1, obj2) and (obj1 not in goals or obj2 not in goals):
                return True
    return False

def is_adjacent(obj1, obj2):
    return abs(obj1[0] - obj2[0]) + abs(obj1[1] - obj2[1]) == 1

def has_horizontal_separation(state, obstacle):
    for y in range(len(range(state.height)) - 1):
        blocked = [0] * state.width
        for x in range(len(range(state.width))):
            if (x, y) in obstacle or (x, y + 1) in obstacle:
                blocked[x] = 1
                
        if all(blocked):
            return y
            break
        
    return False


def has_vertical_separation(state, obstacle):
    for x in range(len(range(state.width)) - 1):
        blocked = [0] * state.height
        for y in range(len(range(state.height))):
            if (x, y) in obstacle or (x + 1, y) in obstacle:
                blocked[y] = 1
                
        if all(blocked):
            return x
            break
        
    return False
            


def is_unreachable_box(box, goal, obstacles):
    
    if box is None:
        return False
    
    
    
    range_x = list(range(min(box[0], goal[0]), max(box[0], goal[0]) + 1))
    range_y = list(range(min(box[1], goal[1]), max(box[1], goal[1]) + 1))
    
    
    
    for i in range(len(range_x) - 1):
        x = range_x[i]
        blocked = [0] * len(range_y)
        for j in range(len(range_y)):
            y = range_y[j]
            if (x, y) in obstacles or (x + 1, y) in obstacles:
                blocked[j] = 1
                
        if all(blocked):
            return x
            break
        
    for i in range(len(range_y) - 1):
        y = range_y[i]
        blocked = [0] * len(range_x)
        for j in range(len(range_x)):
            x = range_x[j]
            if (x, y) in obstacles or (x, y + 1) in obstacles:
                blocked[j] = 1
                
        if all(blocked):
            return y
            break
        
    return False


def has_unreachable_box(pairs, obstacle):
    
    for goal, box in pairs:
            if is_unreachable_box(box, goal, obstacle):
                return True
    return False


def has_unmovable_box(state, obstacle):
    vertical_border = has_vertical_separation(state, obstacle)
    if vertical_border:
        if sum(x for (x,y) in state.boxes if x <= vertical_border) - sum(x for (x,y) in state.storage if x <= vertical_border):
            return True
    horizontal_border = has_horizontal_separation(state, obstacle)
    if horizontal_border:
        if sum(y for (x,y) in state.boxes if y <= horizontal_border) - sum(y for (x,y) in state.storage if y <= horizontal_border):
            return True
    return False
    
        
def find_min_dist_manhattan_path(box, goal, obstacle):
    
    h = goal[0] - box[0]
    v = - (goal[1] - box[1])
    
    if h >= 0:
        hmove = RIGHT
    else:
        hmove = LEFT
    if v >= 0:
        vmove = UP
    else:
        vmove = DOWN
        
    if h == 0 and v == 0:
        return 0
    
    h_blocked = hmove.move(box) in obstacle
    v_blocked = vmove.move(box) in obstacle
    
    
    if v == 0 and h_blocked:
        return math.inf
    if h == 0 and v_blocked:
        return math.inf
    if h_blocked and v_blocked:
        return math.inf

    if h != 0 and (v != 0 or not v_blocked):
        return 1 + find_min_dist_manhattan_path(hmove.move(box), goal, obstacle)
    else:
        return 1 + find_min_dist_manhattan_path(vmove.move(box), goal, obstacle)    


def non_goal_box(state):
    return [box for box in state.boxes if box not in state.storage]

def all_goal_box(boxes, state):
    return all(box in state.storage for box in boxes)

def sum_min_dist_manhattan_path(boxes, goals, obstacle):
    
    distance = [[manhattan(box, goal) for goal in goals] for box in boxes]
    
    for i in range(len(goals) - len(boxes)):
        distance.append([0] * len(goals))
    
    box_permutation = itertools.permutations(list(range(len(goals))))
    goals = list(range(len(goals)))
    
    min_dist = min(sum(distance[boxes[i]][i] for i in goals ) for boxes in box_permutation)
    return min_dist


def minimum_manhattan_pairs_distance(state, obstacles):
    
    goals = list(state.storage)
    boxes = list(state.boxes)
    
    distance = [[manhattan(box, goal) for goal in goals] for box in boxes]
    
    for i in range(len(goals) - len(boxes)):
        distance.append([0] * len(goals))
    
    box_permutation = itertools.permutations(list(range(len(goals))))
    
    min_dist = math.inf
    
    
    for perm in box_permutation:
        cur_dist = sum(distance[perm[i]][i] for i in range(len(goals)))
        if cur_dist < min_dist:
            min_dist = cur_dist
            min_perm = perm
    
    pairs = []
    for i in range(len(min_perm)):
        if min_perm[i] > len(boxes) - 1:
            pairs.append((goals[i], None))
        else:  
            pairs.append((goals[i], boxes[min_perm[i]]))
    
    
    return min_dist, pairs
        
            
    
    

def manhattan(box, goal):
    return abs(goal[0] - box[0]) + abs(goal[1] - box[1])


def has_box_aggregate(state):
    for box in state.boxes:
        u = UP.move(box)
        r = RIGHT.move(box)
        d = DOWN.move(box)
        l = LEFT.move(box)
        if u in state.boxes:
            if r in state.boxes and UP.move(r) in state.boxes and not all_goal_box([box, u, r, UP.move(r)], state):
                return True
            else:
                if l in state.boxes and UP.move(l) in state.boxes and not all_goal_box([box, u, l, UP.move(l)], state):
                    return True
        if d in state.boxes:
            if r in state.boxes and DOWN.move(r) in state.boxes and not all_goal_box([box, d, r, DOWN.move(r)], state):
                return True
            else:
                if l in state.boxes and DOWN.move(l) in state.boxes and not all_goal_box([box, d, l, DOWN.move(l)], state):
                    return True
    return False



def print_state(state):
    print()
    print("=== State ===")
    print("_____________")
    for i in range(state.height):
        
        
        for j in range(state.width):
            if (j, i) in state.storage and (j, i) in state.boxes:
                print('⌊ ✣ ', end = '')
            elif (j, i) in state.storage and (j, i) in state.robots:
                print('⌊ ⍜ ', end = '')
            elif (j, i) in state.boxes:
                print('⌊ ◼︎ ', end = '')
            elif (j, i) in state.storage:
                print('⌊ ◎ ', end = '')
            elif (j, i) in state.obstacles:
                print('⌊❙❙❙', end = '')
            elif (j, i) in state.robots:
                print('⌊ R ', end = '')
            else:
                print('⌊   ', end = '')
        print("⌊")
    print("_____________")
    print("=== State ===")
    print()
    

# if __name__ == "__main__":
    # EXAMPLES OF RUNNING THE SEARCH ALGORITHMS
    # se = SearchEngine('astar', 'full')
    # final = se.search(PROBLEMS[0], sokoban
    # find_min_dist_manhattan_path((0, 0), (4, 5), {(0, 1)})
    
    
    
