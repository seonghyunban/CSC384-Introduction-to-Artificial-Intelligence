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
    return futo_grid[2 * r - 1, 2 * c - 1]

def generate_dom(n):
    return [i + 1 for i in range(n)]

def futoshiki_size(futo_grid):
    return (len(futo_grid) + 1) // 2


###################################################################################################
# Variable Functions
###################################################################################################
def generate_vars(n):
    return [[cspbase.Variable(f"C{i}{j}", generate_dom(n)) for j in range(n)] for i in range(n)]
    
def flatten_vars(nested_vars):
    vars = []
    for row in nested_vars:
        vars.extend(row)
    return vars


###################################################################################################
# Constraint Functions
###################################################################################################
def generate_not_equal_constraint(var_pair):
    con = cspbase.Constraint(f"NotEq({var_pair[0].name},{var_pair[1].name})", [var_pair[0], var_pair[1]])
    con.add_satisfying_tuples([(v1, v2) for (v1, v2) in itertools.product(var_pair[0].domain, var_pair[1].domain) if v1 != v2])
    return con

def binary_not_equal_constraints(n, vars):
    
    cons = []
    for row in vars:
        for pair in itertools.combinations(row, 2):
            cons.append(generate_not_equal_constraint(pair))
            
    for i in range(len(vars)):
        col = [vars[j][i] for j in range(len(vars))]
        for pair in itertools.combinations(col, 2):
            cons.append(generate_not_equal_constraint(pair))
    
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
    
    vars = generate_vars(n)
    cons = binary_not_equal_constraints(n, vars)
    csp = generate_csp(n, vars, cons)
    
    return csp, vars


def futoshiki_csp_model_2(futo_grid):
    ##IMPLEMENT
    pass