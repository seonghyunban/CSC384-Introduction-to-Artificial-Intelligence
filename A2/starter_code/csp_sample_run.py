from cspbase import *
from propagators import *
import itertools

# Initialize 4 variables with (name, domain) pairs
x = Variable('X', [1, 2, 3])
y = Variable('Y', [1, 2, 3])
z = Variable('Z', [1, 2, 3])
w = Variable('W', [1, 2, 3, 4])

# Define a constraint checker that w == x + y + z
def w_eq_sum_x_y_z(wxyz):
    #note inputs lists of value
    w = wxyz[0]
    x = wxyz[1]
    y = wxyz[2]
    z = wxyz[3]
    return(w == x + y + z)


c1 = Constraint('C1', [x, y, z])
#c1 is constraint x == y + z. Below are all of the satisfying tuples
c1.add_satisfying_tuples([[2, 1, 1], [3, 1, 2], [3, 2, 1]])

c2 = Constraint('C2', [w, x, y, z])
#c2 is constraint w == x + y + z. Instead of writing down the satisfying
#tuples we compute them

varDoms = []
for v in [w, x, y, z]:
    varDoms.append(v.domain()) # Put the domain of each variable into a list

sat_tuples = []
for t in itertools.product(*varDoms): # Generate cartesian product of the unpacked domains of the variables
    #NOTICE use of * to convert the list v to a sequence of arguments to product
    if w_eq_sum_x_y_z(t):
        sat_tuples.append(t)

c2.add_satisfying_tuples(sat_tuples) # Add the satisfying tuples to the constraint

simpleCSP = CSP("SimpleEqs", [x,y,z,w]) # Create a CSP object with the variables and constraints
simpleCSP.add_constraint(c1)
simpleCSP.add_constraint(c2)

btracker = BT(simpleCSP) # Create a backtracking object with the CSP
#btracker.trace_on()

print("Plain Bactracking on simple CSP")
btracker.bt_search(prop_BT) # Perform backtracking search with plain backtracking
print("=======================================================")
#print("Forward Checking on simple CSP")
#btracker.bt_search(prop_FC)
#print("=======================================================")
#print("GAC on simple CSP")
#btracker.bt_search(prop_GAC)

#Now n-Queens example

def queensCheck(qi, qj, i, j):
    '''Return true if i and j can be assigned to the queen in row qi and row qj 
       respectively. Used to find satisfying tuples.
    '''
    return i != j and abs(i-j) != abs(qi-qj)

def nQueens(n):
    '''Return an n-queens CSP'''
    # Create domain values
    i = 0
    dom = []
    for i in range(n):
        dom.append(i+1)

    # Create variables
    vars = []
    for i in dom:
        vars.append(Variable('Q{}'.format(i), dom))

    # Create constraints for all pairs of queens
    cons = []    
    for qi in range(len(dom)):
        for qj in range(qi+1, len(dom)):
            con = Constraint("C(Q{},Q{})".format(qi+1,qj+1),[vars[qi], vars[qj]]) 
            
            # Take cartesian product of two domain and get the satisfying tuples
            sat_tuples = []
            for t in itertools.product(dom, dom):
                if queensCheck(qi, qj, t[0], t[1]):
                    sat_tuples.append(t)
            con.add_satisfying_tuples(sat_tuples)
            
            cons.append(con)
    
    # Create CSP object
    csp = CSP("{}-Queens".format(n), vars)
    for c in cons:
        csp.add_constraint(c)
    return csp

def solve_nQueens(n, propType, trace=False):
    csp = nQueens(n)
    solver = BT(csp)
    if trace:
        solver.trace_on()
    if propType == 'BT':
        solver.bt_search(prop_BT)
    elif propType == 'FC':
        solver.bt_search(prop_FC)
    elif propType == 'GAC':
        solver.bt_search(prop_GAC)

#trace = True
trace = False
print("Plain Bactracking on 8-queens")
solve_nQueens(8, 'BT', trace)
print("=======================================================")
#print("Forward Checking 8-queens")
#solve_nQueens(8, 'FC', trace)
#print("=======================================================")
#print("GAC 8-queens")
#solve_nQueens(8, 'GAC', trace)

