from __future__ import annotations

from typing import TYPE_CHECKING
from collections import deque
if TYPE_CHECKING:
    from algorithms.problems_csp import DroneAssignmentCSP


def backtracking_search(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    
    
    """
    Basic backtracking search without optimizations.

    Tips:
    - An assignment is a dictionary mapping variables to values (e.g. {X1: Cell(1,2), X2: Cell(3,4)}).
    - Use csp.assign(var, value, assignment) to assign a value to a variable.
    - Use csp.unassign(var, assignment) to unassign a variable.
    - Use csp.is_consistent(var, value, assignment) to check if an assignment is consistent with the constraints.
    - Use csp.is_complete(assignment) to check if the assignment is complete (all variables assigned).
    - Use csp.get_unassigned_variables(assignment) to get a list of unassigned variables.
    - Use csp.domains[var] to get the list of possible values for a variable.
    - Use csp.get_neighbors(var) to get the list of variables that share a constraint with var.
    - Add logs to measure how good your implementation is (e.g. number of assignments, backtracks).

    You can find inspiration in the textbook's pseudocode:
    Artificial Intelligence: A Modern Approach (4th Edition) by Russell and Norvig, Chapter 5: Constraint Satisfaction Problems
    """
    return backtrack(csp,{})
  
def backtrack(csp, assignment)->DroneAssignmentCSP:
  if csp.is_complete(assignment):
      return assignment
  var=csp.get_unassigned_variables(assignment)[0]
  for value in csp.domains[var]: 
    if csp.is_consistent(var, value, assignment):
      csp.assign(var, value, assignment)
      result= backtrack(csp,assignment)
      if result is not None:
        return result
      csp.unassign(var, assignment)
  return None
    
def backtracking_fc(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking search with Forward Checking.

    Tips:
    - Forward checking: After assigning a value to a variable, eliminate inconsistent values from
      the domains of unassigned neighbors. If any neighbor's domain becomes empty, backtrack immediately.
    - Save domains before forward checking so you can restore them on backtrack.
    - Use csp.get_neighbors(var) to get variables that share constraints with var.
    - Use csp.is_consistent(neighbor, val, assignment) to check if a value is still consistent.
    - Forward checking reduces the search space by detecting failures earlier than basic backtracking.
    """
    return backtrack_fc(csp, {})
  
def backtrack_fc(csp,assignment):
  if csp.is_complete(assignment):
      return assignment
  var=csp.get_unassigned_variables(assignment)[0]
  for value in csp.domains[var]: 
    if csp.is_consistent(var, value, assignment):
      csp.assign(var, value, assignment)
      dominios_guardados={}
      for variable in csp.domains:
        dominios_guardados[variable]=list(csp.domains[variable]) #Save domains before forward checking so you can restore them on backtrack
      failure = False
      for neighbor in csp.get_neighbors(var): #Use csp.get_neighbors(var) to get variables that share constraints with var.
        if neighbor not in assignment:
          nuevo_dominio = []
          for val in csp.domains[neighbor]:
            if csp.is_consistent(neighbor, val, assignment): #Use csp.is_consistent(neighbor, val, assignment) to check if a value is still consistent.
              nuevo_dominio.append(val)
          csp.domains[neighbor]=nuevo_dominio
          if len(nuevo_dominio)==0:
              failure = True
      if failure is not True:
        resultado = backtrack_fc(csp,assignment)
        if resultado is not None:
          return resultado
      csp.domains = dominios_guardados
      csp.unassign(var, assignment) 
  return None


def backtracking_ac3(csp: DroneAssignmentCSP) -> dict[str, str] | None:

    if not arc3(csp):
        return None
    return backtracking_search(csp)
  
def arc3(csp)->bool:
    queue = deque()
    for xi in csp.variables:
        for xj in csp.get_neighbors(xi):
            queue.append((xi, xj))
    while queue:
        xi, xj = queue.popleft()
        if revise(xi, xj, csp):
            if not csp.domains[xi]: 
                return False
            for xk in csp.get_neighbors(xi):
                if xk != xj:
                    queue.append((xk, xi))
    return True
  
def revise(xi, xj, csp):
    revised = False
    for a in list(csp.domains[xi]):
        supported = False
        for b in csp.domains[xj]:
            if pair_consistent(csp, xi, a, xj, b):
                supported = True
                break
        if not supported:
            csp.domains[xi].remove(a)
            revised = True

    return revised
  
def pair_consistent(csp, xi, a, xj, b):

    if a != b:
        return True
    drone = csp.drones[a]
    dpi = csp.var_to_delivery[xi]
    dpj = csp.var_to_delivery[xj]
    if dpi["weight"] + dpj["weight"] > drone["capacity"]:
        return False
    pos = drone["position"]
    dist_i = csp._get_distance(pos, dpi["position"])
    dist_j = csp._get_distance(pos, dpj["position"])

    route = 2*dist_i + 2*dist_j

    if route > drone["battery"]:
        return False

    return True 
  

def backtracking_mrv_lcv(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking with Forward Checking + MRV + LCV.

    Tips:
    - Combine the techniques from backtracking_fc, mrv_heuristic, and lcv_heuristic.
    - MRV (Minimum Remaining Values): Select the unassigned variable with the fewest legal values.
      Tie-break by degree: prefer the variable with the most unassigned neighbors.
    - LCV (Least Constraining Value): When ordering values for a variable, prefer
      values that rule out the fewest choices for neighboring variables.
    - Use csp.get_num_conflicts(var, value, assignment) to count how many values would be ruled out for neighbors if var=value is assigned.
    """
    # TODO: Implement your code here (BONUS)
    return None
  
  #ABREVIATURAS PARA LAS FLAGS
  
bts=backtracking_search
ac3=backtracking_ac3
mrv=backtracking_mrv_lcv
fc=backtracking_fc
  
