#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
import math
import numpy as np
from numpy.linalg import norm
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count
  
def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    '''useful functions: dict_keys() -> returns all keys ''' 
    #positions -> s.storage, s.boxes 
    box_location = np.array(list(state.boxes.keys()))
    storage_location = np.array(list(state.storage.keys())) 
    
    box_number_temp = state.boxes.values()
    box_number_temp = list(box_number_temp) 
    box_number = []
    for i in range(0,len(box_number_temp)):
      box_number.append([]) 
      box_number[i] = box_number[i] + [box_number_temp[i]]
    box_number = np.array(box_number) 
    
    #store all values for boxes
    Man_distance_array = [] 
   
    for i in range(0,len(box_location)):
      Man_distance_array.append([])
      for j in range(0,len(storage_location)):
        if state.restrictions is not None:
          r = np.array(list(state.restrictions[box_number[i][0]]))
          if np.array_equal(r[0], storage_location[j]) is True:
            Man_distance_array[i] = Man_distance_array[i] + [box_location[i] - storage_location[j]]
        elif state.restrictions is None:
          Man_distance_array[i] = Man_distance_array[i] + [box_location[i] - storage_location[j]]
        
    #Man_distance_array is an array which contains each possibile designation of the boxes with restrictions
    Man_distance = [] 
   # print(Man_distance_array)
    Man_distance_array = np.abs(Man_distance_array)
    for i in range(0,len(Man_distance_array)):
      Man_distance.append([])
      for j in range(0,len(Man_distance_array[i])):
        NORM = norm(Man_distance_array[i][j])
        Man_distance[i] = Man_distance[i] + [NORM] 
    
    Man_distanceindex = [] 
    Man_distance2 = np.zeros((2,))
    for i in range(0, len(Man_distance)):
      index = np.argmin(Man_distance[i])
      Man_distanceindex.append(index)
      Man_distance2 += Man_distance_array[i][Man_distanceindex[i]]
    
    Official_Man_distance = Man_distance2[0] + Man_distance2[1]
    return Official_Man_distance
    
def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    '''useful functions: dict_keys() -> returns all keys ''' 
    box_location = np.array(list(state.boxes.keys()))
    storage_location = np.array(list(state.storage.keys())) 
    obstacle_location = np.array(list(state.obstacles))
    w = state.width
    h = state.height
    inf = float('inf')
    
    box_number_temp = state.boxes.values()
    box_number_temp = list(box_number_temp) 
    box_number = []
    for i in range(0,len(box_number_temp)):
      box_number.append([]) 
      box_number[i] = box_number[i] + [box_number_temp[i]]
    box_number = np.array(box_number) 
    
    box_location_up = box_location + [0,-1]
    box_location_down = box_location + [0,1]
    box_location_right = box_location + [1,0]
    box_location_left = box_location + [-1,0]
    
    #store all values for boxes
    Man_distance_array = [] 
    if np.any(np.in1d(box_location_up[:,0], obstacle_location[:,0])) and np.any(np.in1d(box_location_up[:,1], obstacle_location[:,1])):
      if np.any(np.in1d(box_location_left[:,0], obstacle_location[:,0])) and np.any(np.in1d(box_location_left[:,1], obstacle_location[:,1])):
        return inf
    elif np.any(np.in1d(box_location_up[:,0], obstacle_location[:,0])) and np.any(np.in1d(box_location_up[:,1],obstacle_location[:,1])):
      if np.any(np.in1d(box_location_right[:,0], obstacle_location[:,0])) and  np.any(np.in1d(box_location_right[:,1], obstacle_location[:,1])):
        return inf
    elif np.any(np.in1d(box_location_down[:,0], obstacle_location[:,0])) and np.any(np.in1d(box_location_down[:,1], obstacle_location[:,1])):
      if np.any(np.in1d(box_location_right[:,0], obstacle_location[:,0])) and  np.any(np.in1d(box_location_right[:,1], obstacle_location[:,1])):
        return inf
    elif np.any(np.in1d(box_location_down[:,0], obstacle_location[:,0])) and np.any(np.in1d(box_location_down[:,1],obstacle_location[:,1])):
      if np.any(np.in1d(box_location_right[:,0], obstacle_location[:,0])) and np.any(np.in1d(box_location_right[:,1],obstacle_location[:,1])):
        return inf
    if np.any(box_location_up[:,1] < 0) and np.any(box_location_left[:,0] < 0):
      return inf
    elif np.any(box_location_up[:,1] < 0) and np.any(box_location_right[:,0] > (w-1)):
      return inf 
    elif np.any(box_location_down[:,1] > (h-1)) and np.any(box_location_left[:,0] < 0):
      return inf
    elif np.any(box_location_down[:,1] > (h-1)) and np.any(box_location_right[:,0] > (w-1)):
      return inf
    elif np.any(np.in1d(box_location_up, obstacle_location)) and np.any(box_location_left[:,0] < 0):
      return inf
    elif np.any(np.in1d(box_location_up, obstacle_location)) and np.any(box_location_right[:,0] > (w-1)):
      return inf
    elif np.any(np.in1d(box_location_down, obstacle_location)) and np.any(box_location_left[:,0] < 0):
      return inf
    elif np.any(np.in1d(box_location_down, obstacle_location)) and np.any(box_location_right[:,0] > (w-1)):
      return inf
    elif np.any(np.in1d(box_location_right, obstacle_location)) and np.any(box_location_down[:,1] > (h-1)):
      return inf
    elif np.any(np.in1d(box_location_left, obstacle_location)) and np.any(box_location_down[:,1] > (h-1)):
      return inf
    else:
      pass
      
#store all values for boxes
    Man_distance_array = [] 
    storage_location2 = []
   
    for i in range(0,len(box_location)):
      Man_distance_array.append([])
      storage_location2.append([])
      for j in range(0,len(storage_location)):
        if state.restrictions is not None:
          r = np.array(list(state.restrictions[box_number[i][0]]))
          if np.array_equal(r[0], storage_location[j]) is True:
            Man_distance_array[i] = Man_distance_array[i] + [box_location[i] - storage_location[j]]
            storage_location2[i] = storage_location2[i] + [storage_location[j]]
        elif state.restrictions is None:
          Man_distance_array[i] = Man_distance_array[i] + [box_location[i] - storage_location[j]]
          storage_location2[i] = storage_location2[i] + [storage_location[j]]
    
    #Man_distance_array is an array which contains each possibile designation of the boxes with restrictions
    Man_distance = [] 
    Man_distance_array = np.abs(Man_distance_array)
    for i in range(0,len(Man_distance_array)):
      Man_distance.append([])
      for j in range(0,len(Man_distance_array[i])):
        NORM = norm(Man_distance_array[i][j])
        Man_distance[i] = Man_distance[i] + [NORM] 
    
    Man_distanceindex = []
    for i in range(0, len(Man_distance)):
      index = np.argmin(Man_distance[i])
      Man_distanceindex.append(index)
      
    #We refine the MANHATTAN DISTANCE by considering the position of the robot. Our naive 
    #attempt of the originial MANHATTEN DISTANCE is admissible. 
    Man_distance2 = []
    official_storage_location = [] 
    for i in range(0,len(Man_distance)):
      Man_distance2.append([])
      official_storage_location.append([])
      Man_distance2[i] = Man_distance2[i] + [Man_distance_array[i][Man_distanceindex[i]]]
      official_storage_location[i] = official_storage_location[i] + [storage_location2[i][Man_distanceindex[i]]]
    Man_distance2 = np.array(Man_distance2)
    official_storage_location = np.array(official_storage_location)

    robot_location = np.array(state.robot)
    rel_distance = []
    robot_distance = []
    index_storage = []
    
    for i in range(0,len(box_location)):
      robot_distance.append([])
      rel_distance.append([]) 
      for j in range(0,len(box_location)):
        rel_distance[i] = rel_distance[i] + [norm(box_location[j] - robot_location)]
      if i == 0:  
        min_val_index = (rel_distance[i]).index(min(rel_distance[i]))
        index_storage.append(min_val_index)
      elif i > 0: 
        min_val_index = (rel_distance[i-1]).index(min(rel_distance[i-1]))
        index_storage.append(min_val_index)
        
      for k in range(0,len(index_storage)):
        rel_distance[i][index_storage[k]] = 100000
        
      robot_distance[i] = robot_distance[i] + [box_location[min_val_index] - robot_location]
      robot_location = official_storage_location[min_val_index][0]
      
    robot_distance = np.abs(robot_distance) 
    refined_dis = robot_distance + Man_distance2
    refined_Man_distance = np.sum(refined_dis)
    
    return refined_Man_distance

    
def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    
    fval = sN.gval + weight*sN.hval 
    
    
    return fval

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    time_bound = timebound 
    inf = float('inf')
    costbound = (inf,inf,inf)
    Search = SearchEngine(strategy = 'best_first',cc_level = 'full')
    goal_fn = sokoban_goal_state
    Search.init_search(initial_state, goal_fn, heur_fn)
    FINAL_VAL = False 

    while time_bound > 0: 
      
      T1 = os.times()[0]
      Final = Search.search(time_bound,costbound)
      T2 = os.times()[0]
      T_diff = T2 - T1 
      time_bound -= T_diff
      
      if Final is not False:
        costbound = (Final.gval,inf, inf)
        FINAL_VAL = Final
      elif Final is False:
        break 
        
    return FINAL_VAL
  

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    
    time_bound = timebound 
    inf = float('inf')
    costbound = (inf,inf,inf)
    Search = SearchEngine(strategy = 'astar',cc_level = 'full')
    goal_fn = sokoban_goal_state
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    Search.init_search(initial_state, goal_fn, heur_fn,fval_function)
    FINAL_VAL = False

    while time_bound > 0: 
      
      T1 = os.times()[0]
      Final = Search.search(time_bound,costbound)
      T2 = os.times()[0]
      T_diff = T2 - T1 
      time_bound -= T_diff
      
      if Final is not False:
        costbound = (Final.gval,inf, inf)
        FINAL_VAL = Final
      elif Final is False:
        break 

    return FINAL_VAL

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 



