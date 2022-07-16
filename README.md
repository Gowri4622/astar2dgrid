# A* Path Finding Algorithm for 2D Grid World
## AIM

To develop a code to find the route from the source to the destination point using A* algorithm for 2D grid world.

## THEORY
We try to use the A* algorithm to navigate through a 2D Gird environment. We provide the algorithm with the inital and goal states, and then let the algorithm calculate the Heuristic function to decide the path nodes. And finally, we return the path nodes to the user.

## DESIGN STEPS

### STEP 1:
Build a 2D grid world with initial state and goal state

Initial State: (2,2)

Goal State: (5,8)

### STEP 2:
Mention the Obstacles in the 2D grid World

### STEP 3:
Define the function for the distance function for the heuristic function

### STEP 4:
Pass all the values to the GirdProblem, and print the solution path.



## Draw the 2D 
![Screenshot_645](https://user-images.githubusercontent.com/75235455/168839044-4326a25a-c159-4fbf-bc90-d365b664415b.png)


## PROGRAM
```python

def best_first_search(problem, f):
    "Search nodes with minimum f(node) value first."
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
    return failure

def g(n): 
    return n.path_cost
```

### 2D Grid Pathfinding Problem
```python
class GridProblem(Problem):
    """Finding a path on a 2D grid with obstacles. Obstacles are (x, y) cells."""

    def __init__(self, initial=(15, 30), goal=(130, 30), obstacles=(), **kwds):
        Problem.__init__(self, initial=initial, goal=goal, 
                         obstacles=set(obstacles) - {initial, goal}, **kwds)

    directions = [(-1, -1), (0, -1), (1, -1),
                  (-1, 0),           (1,  0),
                  (-1, +1), (0, +1), (1, +1)]
    
    def action_cost(self, s, action, s1): 
        return straight_line_distance(s, s1)
    
    def h(self, node): 
        return straight_line_distance(node.state, self.goal)
                  
    def result(self, state, action): 
        "Both states and actions are represented by (x, y) pairs."
        return action if action not in self.obstacles else state
    
    def actions(self, state):
        """You can move one cell in any of `directions` to a non-obstacle cell."""
        
        x, y = state
        return {(x + dx, y + dy) for (dx, dy) in self.directions} - self.obstacles
   

def straight_line_distance(A, B):
    "Straight-line distance between two points."
 
    return sum(abs(a - b)**2 for (a, b) in zip(A, B)) ** 0.5

def g(n): 
    
    return n.path_cost

def astar_search(problem, h=None):
    """Search nodes with minimum f(n) = g(n) + h(n)."""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + h(n))

obstacles={(8,1),(7,2),(7,3),(2,4),(4,2),(4,4),(5,4),(6,4),(8,4),(5,5),(2,6),(2,7),(4,7),(5,7),(6,7),(7,8)}

grid1 = GridProblem(initial=(1,2), goal =(8,6) ,obstacles=obstacles)

solution1 = astar_search(grid1)

path_states(solution1)

```


## OUTPUT:
![Screenshot_646](https://user-images.githubusercontent.com/75235455/168839703-482ca525-8667-4a46-b260-6da81d171af3.png)

The algorithm is able to find the solution path for the given problem. But the solution path, might not be the shortest path to reach the goal state.

## RESULT:
Hence, A* Algorithm was implemented to find the route from the source to the destination point in a 2D gird World.

