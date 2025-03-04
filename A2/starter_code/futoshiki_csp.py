# Look for #IMPLEMENT tags in this file.
'''
All models need to return a CSP object, and a list of lists of Variable objects
representing the board. The returned list of lists is used to access the
solution.

For example, after these three lines of code

    csp, var_array = futoshiki_csp_model_1(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the Futoshiki puzzle.

1. futoshiki_csp_model_1 (worth 20/100 marks)
    - A model of a Futoshiki grid built using only
      binary not-equal constraints for both the row and column constraints.

2. futoshiki_csp_model_2 (worth 20/100 marks)
    - A model of a Futoshiki grid built using only n-ary
      all-different constraints for both the row and column constraints.
    
    The input board is specified as a list of n lists. Each of the n lists
    represents a row of the board. If a 0 is in the list it represents an empty
    cell. Otherwise if a number between 1--n is in the list then this
    represents a pre-set board position.

    Each list is of length 2n-1, with each space on the board being separated
    by the potential inequality constraints. '>' denotes that the previous
    space must be bigger than the next space; '<' denotes that the previous
    space must be smaller than the next; '.' denotes that there is no
    inequality constraint.

    E.g., the board

    -------
    | > |2|
    | | | |
    | | < |
    -------
    would be represented by the list of lists

    [[0,>,0,.,2],
     [0,.,0,.,0],
     [0,.,0,<,0]]

'''
import cspbase
import itertools


###################################################################################################
# Futogrid Functions
###################################################################################################
def cell_of(futo_grid, r, c):
    return futo_grid[2 * r][c]

def generate_dom(n):
    return [i + 1 for i in range(n)]

def futoshiki_size(futo_grid):
    return (len(futo_grid))


###################################################################################################
# Variable Functions
###################################################################################################
def generate_vars(n, futo_grid):
    
    vars = []
    
    for i in range(n):
        row = []
        for j in range(n):
            cell = cell_of(futo_grid, i, j)
            if cell != 0:
                row.append(cspbase.Variable(f"C{i}{j}", [cell]))
            else:
                row.append(cspbase.Variable(f"C{i}{j}", generate_dom(n)))
        vars.append(row)
                        
    return vars
    
def flatten_vars(nested_vars):
    vars = []
    for row in nested_vars:
        vars.extend(row)
    return vars


###################################################################################################
# Constraint Functions
###################################################################################################
def binary_not_equal_constraint(row):
    
    cons = []
    for var_pair in itertools.combinations(row, 2):
        con = cspbase.Constraint(f"NotEq({var_pair[0].name},{var_pair[1].name})", [var_pair[0], var_pair[1]])
        con.add_satisfying_tuples([(v1, v2) for (v1, v2) in itertools.product(var_pair[0].domain(), var_pair[1].domain()) if v1 != v2])
        cons.append(con)
        
    return cons


def generate_binary_not_equal_constraints(n, vars):
    
    cons = []
    for row in vars:
        cons.extend(binary_not_equal_constraint(row))
            
    for i in range(n):
        col = [vars[j][i] for j in range(n)]
        cons.extend(binary_not_equal_constraint(col))
    
    return cons


def all_diff_constraint(n, seq_type, pos, sequence):
    
    con = cspbase.Constraint(f"AllDiff({seq_type}{pos})", sequence)
    con.add_satisfying_tuples([tup for tup in itertools.product(*(var.domain() for var in sequence)) if len(set(tup)) == len(tup)])
    return con


def generate_all_diff_constraint(n, vars):
    
    cons = []
    for r in range(n):
        row = vars[r]
        cons.append(all_diff_constraint(n, "R", r, row))
            
    for c in range(n):
        col = [vars[j][c] for j in range(n)]
        cons.append(all_diff_constraint(n, "C", c, col))
    
    return cons


def inequality_constraint(type, var_left, var_right):
    
    if type == ">":
        con = cspbase.Constraint(f"GT({var_left.name}{var_right.name})", [var_left, var_right])
        con.add_satisfying_tuples([(v1, v2) for (v1, v2) in itertools.product(var_left.domain(), var_right.domain()) if v1 > v2])

    else:
        con = cspbase.Constraint(f"LT({var_left.name}{var_right.name})", [var_left, var_right])
        con.add_satisfying_tuples([(v1, v2) for (v1, v2) in itertools.product(var_left.domain(), var_right.domain()) if v1 < v2])
    
    return con


def generate_inequality_constraints(n, futogrid, vars):
    
    cons = []
    for row in range(n):
        for col in range(n - 1):
            type = futogrid[row][2 * col + 1]
            if type in [">", "<"]:
                var_left = vars[row][col]
                var_right = vars[row][col + 1]
                cons.append(inequality_constraint(type, var_left, var_right))
                
    return cons
        
    


###################################################################################################
# CSP Functions
###################################################################################################
def generate_csp(n, vars, cons):
    
    csp = cspbase.CSP(f"{n}-Futoshiki")
    
    for i in range(n):
        for j in range(n):
            csp.add_var(vars[i][j])
    
    for con in cons:
        csp.add_constraint(con)
    
    return csp
    

###################################################################################################
# Futoshiki Models
###################################################################################################
def futoshiki_csp_model_1(futo_grid):
    n = futoshiki_size(futo_grid)
    
    vars = generate_vars(n, futo_grid)
    cons = generate_binary_not_equal_constraints(n, vars)
    cons.extend(generate_inequality_constraints(n, futo_grid, vars))
    csp = generate_csp(n, vars, cons)
    
    return csp, vars


def futoshiki_csp_model_2(futo_grid):
    n = futoshiki_size(futo_grid)
    
    vars = generate_vars(n, futo_grid)
    cons = generate_all_diff_constraint(n, vars)
    cons.extend(generate_inequality_constraints(n, futo_grid, vars))
    csp = generate_csp(n, vars, cons)
    
    return csp, vars