# Look for #IMPLEMENT tags in this file. These tags indicate what has
# to be implemented to complete problem solution.

'''This file will contain different constraint propagators to be used within 
   bt_search.

   propagator == a function with the following template
      propagator(csp, newly_instantiated_variable=None)
           ==> returns (True/False, [(Variable, Value), (Variable, Value) ...]

      csp is a CSP object---the propagator can use this to get access
      to the variables and constraints of the problem. The assigned variables
      can be accessed via methods, the values assigned can also be accessed.

      newly_instaniated_variable is an optional argument.
      if newly_instantiated_variable is not None:
          then newly_instantiated_variable is the most
           recently assigned variable of the search.
      else:
          progator is called before any assignments are made
          in which case it must decide what processing to do
           prior to any variables being assigned. SEE BELOW

       The propagator returns True/False and a list of (Variable, Value) pairs.
       Return is False if a deadend has been detected by the propagator.
       in this case bt_search will backtrack
       return is true if we can continue.

      The list of variable values pairs are all of the values
      the propagator pruned (using the variable's prune_value method). 
      bt_search NEEDS to know this in order to correctly restore these 
      values when it undoes a variable assignment.

      NOTE propagator SHOULD NOT prune a value that has already been 
      pruned! Nor should it prune a value twice

      PROPAGATOR called with newly_instantiated_variable = None
      PROCESSING REQUIRED:
        for plain backtracking (where we only check fully instantiated 
        constraints) 
        we do nothing...return true, []

        for forward checking (where we only check constraints with one
        remaining variable)
        we look for unary constraints of the csp (constraints whose scope 
        contains only one variable) and we forward_check these constraints.

        for gac we establish initial GAC by initializing the GAC queue
        with all constaints of the csp

      PROPAGATOR called with newly_instantiated_variable = a variable V
      PROCESSING REQUIRED:
         for plain backtracking we check all constraints with V (see csp method
         get_cons_with_var) that are fully assigned.

         for forward checking we forward check all constraints with V
         that have one unassigned variable left

         for gac we initialize the GAC queue with all constraints containing V.
		 

var_ordering == a function with the following template
    var_ordering(csp)
        ==> returns Variable 

    csp is a CSP object---the heuristic can use this to get access to the
    variables and constraints of the problem. The assigned variables can be
    accessed via methods, the values assigned can also be accessed.

    var_ordering returns the next Variable to be assigned, as per the definition
    of the heuristic it implements.
   '''

def prop_BT(csp, newVar=None):
    '''Do plain backtracking propagation. That is, do no 
    propagation at all. Just check fully instantiated constraints'''

    # At the root level, we can continue and nothing is pruned.
    if not newVar:
        return True, []
    
    # At the child level, we need to check if the new variable assignment
    # Get all constraints where all variable within its scope is all assigned, that the new variable is involved.
    
    for c in csp.get_cons_with_var(newVar):
        if c.get_n_unasgn() == 0: # If all variables are assigned for the constraint.
            vars = c.get_scope() # Get all variables in the scope of the constraint.
            vals = [] # 
            for var in vars:
                vals.append(var.get_assigned_value()) # Get the assigned value of those assigned variables.
            if not c.check(vals): # And check if they satisfy the constraint.
                return False, []
            
    return True, []

def prop_FC(csp, newVar=None):
    '''Do forward checking. That is check constraints with
       only one uninstantiated variable. Remember to keep
       track of all pruned variable,value pairs and return '''

    pruned = []
    DWO = False

    # At the root level, we check all unary constraints.
    if not newVar:
        for c in csp.get_all_cons():
            if len(c.get_scope()) == 1 and c.get_unasgn_vars(): # Find unary constraints that has not yet been assigned.
                
                unary_var = c.get_unasgn_vars()[0] # Get the unassigned variable in the scope of the constraint.
                for val in unary_var.cur_domain():
                    if not c.check([val]):
                        unary_var.prune_value(val)
                        pruned.append((unary_var, val))
                
                if unary_var.cur_domain_size() == 0: # When we reach DWO, we return False and pruned values (for domain restore).
                    DWO = True

    # At the child level, we need to check all the constraints that is related to the newly assigned variable.
    else:
        for c in csp.get_cons_with_var(newVar):
            if c.get_n_unasgn() == 1: # If only one variables is not assigned for the constraint.
                
                unassigned_var = c.get_unasgn_vars()[0] # Get the unassigned variable in the scope of the constraint.
                scope = c.get_scope()
                
                for val in unassigned_var.cur_domain(): # For each value in the domain of the selected variable.
                    
                    # Create the list val of potential assignment.
                    vals = []
                    for var in scope:
                        if var.is_assigned():
                            vals.append(var.get_assigned_value()) 
                        else:
                            vals.append(val)
                    
                    # Check if the potential assignment is valid and prune the value from the domain if it is not.
                    if not c.check(vals):
                        unassigned_var.prune_value(val)
                        pruned.append((unassigned_var, val))
                
                if unassigned_var.cur_domain_size() == 0: # When we reach DWO, we return False and pruned values (for domain restore).
                    DWO = True

    return not DWO, pruned


def prop_GAC(csp, newVar=None):
    '''Do GAC propagation. If newVar is None we do initial GAC enforce 
    processing all constraints. Otherwise we do GAC enforce with
    constraints containing newVar on GAC Queue'''
    
    # Initialize GAC Queue as specified
    if newVar is None:
        gac_queue = csp.get_all_cons()
    else:
        gac_queue = csp.get_cons_with_var(newVar)
    
    # Pruned variable list
    pruned = []
    
    while gac_queue:
        # Get a new queue
        c = gac_queue[0]
        gac_queue = gac_queue[1:]
        
        # For each (var-val) pair
        for var in c.get_scope():
            for val in var.cur_domain():
                
                # If the pair does not have support, prune it, append to pruned list
                if not c.has_support(var, val):
                    var.prune_value(val)
                    pruned.append((var, val))
                    
                    # If DWO reached, reture false and pruned (for domain restore)
                    if var.cur_domain_size() == 0:
                        return False, pruned
                    # Else, domain modified, and hence enqueue all the constrain associated with the scope of the vars.
                    else:
                        for domain_updated_var in c.get_scope():
                            gac_queue.extend([con for con in csp.get_cons_with_var(domain_updated_var) if con not in gac_queue])

    return True, pruned


def ord_mrv(csp):
    ''' return variable according to the Minimum Remaining Values heuristic '''
    #IMPLEMENT
    
    # Get all unassigned variables.
    unassigned_vars = csp.get_unasgn_vars()
    
    # Declare variables for minimum remaining domain size variable.
    min_size = 0
    min_var = None
    
    # Find minimum
    for var in unassigned_vars:
        curr_size = var.cur_domain_size()
        if curr_size < min_size:
            curr_size = min_size
            min_var = var
    
    return min_var