"""
=== Introduction ===

In this problem, you will again build a planner that helps a robot
  find the best path through a warehouse filled with boxes
  that it has to pick up and deliver to a dropzone. Unlike Part A,
  however, in this problem the robot is moving in a continuous world
  (albeit in discrete time steps) and has constraints on the amount
  it can turn its wheels in a given time step.

Your file must be called `partB.py` and must have a class
  called `DeliveryPlanner`.
This class must have an `__init__` function that takes five
  arguments: `self`, `warehouse`, `todo`, `max_distance`, and
  `max_steering`.
The class must also have a function called `plan_delivery` that
  takes a single argument, `self`.

=== Input Specifications ===

`warehouse` will be a list of m strings, each with n characters,
  corresponding to the layout of the warehouse. The warehouse is an
  m x n grid. warehouse[i][j] corresponds to the spot in the ith row
  and jth column of the warehouse, where the 0th row is the northern
  end of the warehouse and the 0th column is the western end.

The characters in each string will be one of the following:

'.' (period) : traversable space.
'#' (hash) : a wall. If the robot contacts a wall space, it will crash.
'@' (dropzone): the space where all boxes must be delivered. The dropzone may be traversed like
  a '.' space.

Each space is a 1 x 1 block. The upper-left corner of space warehouse[i][j] is at the point (j,-i) in
  the plane. Spaces outside the warehouse are considered walls; if any part of the robot leaves the
  warehouse, it will be considered to have crashed into the exterior wall of the warehouse.

For example,
  warehouse = ['.#.',
               '.#.',
               '..@']
  is a 3x3 warehouse. The dropzone is at space (2,-2) and there are walls at spaces (1,0)
  and (1,-1). The rest of the warehouse is empty space.

The robot is a circle of radius 0.25. The robot begins centered in the dropzone space.
  The robot's initial bearing is 0.

The argument `todo` is a list of points representing the center point of each box.
  todo[0] is the first box which must be delivered, followed by todo[1], and so on.
  Each box is a square of size 0.2 x 0.2. If the robot contacts a box, it will crash.

The arguments `max_distance` and `max_steering` are parameters constraining the movement
  of the robot on a given time step. They are described more below.

=== Rules for Movement ===

- The robot may move any distance between 0 and `max_distance` per time step.
- The robot may set its steering angle anywhere between -`max_steering` and
  `max_steering` per time step. A steering angle of 0 means that the robot will
  move according to its current bearing. A positive angle means the robot will
  turn counterclockwise by `steering_angle` radians; a negative steering_angle
  means the robot will turn clockwise by abs(steering_angle) radians.
- Upon a movement, the robot will change its steering angle instantaneously to the
  amount indicated by the move, and then it will move a distance in a straight line in its
  new bearing according to the amount indicated move.
- The cost per move is 1 plus the amount of distance traversed by the robot on that move.

- The robot may pick up a box whose center point is within 0.5 units of the robot's center point.
- If the robot picks up a box, it incurs a total cost of 2 for that move (this already includes
  the 1-per-move cost incurred by the robot).
- While holding a box, the robot may not pick up another box.
- The robot may put a box down at a total cost of 1.5 for that move. The box must be placed so that:
  - The box is not contacting any walls, the exterior of the warehouse, any other boxes, or the robot
  - The box's center point is within 0.5 units of the robot's center point
- A box is always oriented so that two of its edges are horizontal and the other two are vertical.
- If a box is placed entirely within the '@' space, it is considered delivered and is removed from the
  warehouse.
- The warehouse will be arranged so that it is always possible for the robot to move to the
  next box on the todo list without having to rearrange any other boxes.

- If the robot crashes, it will stop moving and incur a cost of 100*distance, where distance
  is the length it attempted to move that move. (The regular movement cost will not apply.)
- If an illegal move is attempted, the robot will not move, but the standard cost will be incurred.
  Illegal moves include (but are not necessarily limited to):
    - picking up a box that doesn't exist or is too far away
    - picking up a box while already holding one
    - putting down a box too far away or so that it's touching a wall, the warehouse exterior,
      another box, or the robot
    - putting down a box while not holding a box

=== Output Specifications ===

`plan_delivery` should return a LIST of strings, each in one of the following formats.

'move {steering} {distance}', where '{steering}' is a floating-point number between
  -`max_steering` and `max_steering` (inclusive) and '{distance}' is a floating-point
  number between 0 and `max_distance`

'lift {b}', where '{b}' is replaced by the index in the list `todo` of the box being picked up
  (so if you intend to lift box 0, you would return the string 'lift 0')

'down {x} {y}', where '{x}' is replaced by the x-coordinate of the center point of where the box
  will be placed and where '{y}' is replaced by the y-coordinate of that center point
  (for example, 'down 1.5 -2.9' means to place the box held by the robot so that its center point
  is (1.5,-2.9)).

=== Grading ===

- Your planner will be graded against a set of test cases, each equally weighted.
- Each task will have a "baseline" cost. If your set of moves results in the task being completed
  with a total cost of K times the baseline cost, you will receive 1/K of the credit for the
  test case. (Note that if K < 1, this means you earn extra credit!)
- Otherwise, you will receive no credit for that test case. This could happen for one of several
  reasons including (but not necessarily limited to):
  - plan_delivery's moves do not deliver the boxes in the correct order.
  - plan_delivery's output is not a list of strings in the prescribed format.
  - plan_delivery does not return an output within the prescribed time limit.
  - Your code raises an exception.

=== Additional Info ===

- You may add additional classes and functions as needed provided they are all in the file `partB.py`.
- Your partB.py file must not execute any code when it is imported.
- Upload partB.py to Canvas in the Assignments section. Do not put it into an
  archive with other files.
- Ask any questions about the directions or specifications on Piazza.
"""
import math

class DeliveryPlanner:
    warehouse = []
    todo = []


    def __init__(self, warehouse, todo, max_distance, max_steering):

        # initialize values
        self.warehouse = warehouse
        self.todo = todo
        self.max_steering = max_steering
        self.max_distance = max_distance
        pass

    def plan_delivery(self):

        moves = []

        delta = [[-1, 0 ], # go up
                 [ 0, -1], # go left
                 [ 1, 0 ], # go down
                 [ 0, 1 ]]#, # go right
                 #[ 1, 1 ], # go down and right
                 #[-1, -1], # go up and left
                 #[-1, 1 ], # go up and right
                 #[ 1, -1]] # go up and left

        #make expanded warehouse
        new_warehouse_x = len(self.warehouse[0]) * 10
        new_warehouse_y = len(self.warehouse) * 10

        e_warehouse = [[0 for col in range(new_warehouse_x)] for row in range(new_warehouse_y)]

        #initialize starting x and y (changed immediately)
        starting_x = 0.0
        starting_y = 0.0
        robot_initial_i = 3

        #create buffer grid and set starting x and starting y
        starting_x, starting_y = self.buffer(e_warehouse)
        starting_x_coord, starting_y_coord = starting_x, starting_y
        dropoff_x_coord = starting_x_coord + 0.35
        dropoff_y_coord = starting_y_coord

        #place boxes
        x_coord = 0
        y_coord = 0
        counter = 2
        for i in range(len(self.todo)):
            x_coord, y_coord = self.todo[i]

            x_coord = int(x_coord * 10) - 1
            y_coord = int(y_coord * -10) - 1

            counter += 1
            #set to impass with buffer
            for j in range(-3, 5):
                for p in range(-3, 5):
                    if (y_coord + j) >= 0 and (y_coord + j) < len(e_warehouse) and (x_coord + p) >= 0 and (x_coord + p) < len(e_warehouse[0]):
                        if e_warehouse[y_coord+j][x_coord+p] != 1:
                            e_warehouse[y_coord+j][x_coord+p] = counter+1

                #set to only cross section is passable
                for j in range(4):
                    for p in range(len(delta)):
                        if (y_coord + j) >= 0 and (y_coord + j) < len(e_warehouse) and (x_coord + p) >= 0 and (x_coord + p) < len(e_warehouse[0]):
                            if e_warehouse[y_coord+j*delta[p][0]][x_coord+j*delta[p][1]] != 1:
                                y2 = y_coord + delta[p][0] * j
                                x2 = x_coord + delta[p][1] * j
                                e_warehouse[y2][x2] = counter

            #finishing touches
            if e_warehouse[y_coord+4][x_coord] != 1:
                e_warehouse[y_coord + 4][x_coord] = counter
            if e_warehouse[y_coord][x_coord+4] != 1:
                e_warehouse[y_coord][x_coord+4] = counter


        #get starting and ending point
        starting_y = int(starting_y * -10) - 1
        starting_x = int(starting_x * 10) - 1

        #for each box
        for k in range(len(self.todo)):
            #set goal cells
            x_coord, y_coord = self.todo[k]
            x_cell = int(x_coord * 10) - 1
            y_cell = int(y_coord * -10) -1

            #make heuristic for each
            heuristic = self.heuristic(y_cell, x_cell, e_warehouse)

            #make expansion expand grid
            expand_grid = self.expander(starting_y, starting_x, heuristic, x_cell, y_cell, delta, k, e_warehouse)

            x = x_cell
            y = y_cell

            closed = [[0 for col in range(len(e_warehouse[0]))] for row in range(len(e_warehouse))]
            path = [[0 for col in range(len(e_warehouse[0]))] for row in range(len(e_warehouse))]

            #help visualize path by marking obstacles, this can be deleted
            for i in range(len(expand_grid)):
                for j in range(len(expand_grid[0])):
                    if expand_grid[i][j] == -1:
                        path[i][j] = -1
            path[starting_y][starting_x] = 1

            #function to map out path
            found = False
            #set a penalty for changing direction
            penalty = 30
            first_round = True
            #counterino = 0

            while not found:
                current_low = 99999
                for i in range(len(delta)):
                    y2 = y + delta[i][0]
                    x2 = x + delta[i][1]
                    penalty_on = 0
                    #check if you're back at starting point
                    if x2 == starting_x and y2 == starting_y:
                        found = True
                        path[y][x] = expand_grid[y][x]
                    else:
                        if expand_grid[y2][x2] <= expand_grid[y][x]:
                            #do not look for past direction if it is the first round
                            if first_round:
                                if y2 >= 0 and y2 < len(e_warehouse) and x2 >=0 and x2 < len(e_warehouse[0]):
                                    if closed[y2][x2] == 0:
                                        if expand_grid[y2][x2] >= 0 and expand_grid[y2][x2] < current_low:
                                            current_low = expand_grid[y2][x2]
                                            lowest_x = x2
                                            lowest_y = y2
                                            current_last_i = i
                            else:
                                if last_i != i:
                                    penalty_on = 1
                                if y2 >= 0 and y2 < len(e_warehouse) and x2 >=0 and x2 < len(e_warehouse[0]):
                                    if closed[y2][x2] == 0:
                                        total_cost = expand_grid[y2][x2] + penalty_on*penalty
                                        if expand_grid[y2][x2] >= 0 and total_cost < current_low:
                                            current_low = total_cost
                                            lowest_x = x2
                                            lowest_y = y2
                                            current_last_i = i
                if not found:
                    closed[y][x] = 1
                    path[y][x] = expand_grid[y][x]
                    x = lowest_x
                    y = lowest_y
                    last_i = current_last_i
                    first_round = False


#*****************************************************************************
# Method that seeks to create path from initial to final
#*****************************************************************************
            closed = [[0 for col in range(len(e_warehouse[0]))] for row in range(len(e_warehouse))]
            x = starting_x
            y = starting_y
            closed[y][x] = 1
            current_moves = []
            angles = []
            dists = []

            prev_pivot_x = starting_x_coord
            prev_pivot_y = starting_y_coord

            found = False
            last_i = robot_initial_i
            first_round = True
            prev_theta = 0
            while not found:
                current_high = 0
                theta = 0
                for i in range(len(delta)):
                    y2 = y + delta[i][0]
                    x2 = x + delta[i][1]
                    #check if you're within 0.5 of ending cell
                    if y2 >= 0 and y2 < len(e_warehouse) and x2 >=0 and x2 < len(e_warehouse[0]):
                        if closed[y2][x2] == 0:
                            #if this cell is the largest
                            if path[y2][x2] > current_high:
                                current_x_coord = (float(x2) + 1.0) / 10.0
                                current_y_coord = (float(y2) + 1.0) / -10.0
                                dist = math.sqrt((current_x_coord - x_coord)**2 + (current_y_coord - y_coord)**2)
                                if dist < 0.5:
                                    found = True
                                    final_x = x2
                                    final_y = y2
                                else:
                                    #if not within 0.5
                                    current_high = path[y2][x2]
                                    highest_x = x2
                                    highest_y = y2
                                    current_last_i = i
                                    if first_round:
                                        #calculate the direction the robot will face when it returns to starting point
                                        robot_initial_i = (i + 2) % 4
                if not found:
                    closed[y][x] = 1
                    max_dist_reached = 0
                    #if direction has changed
                    if last_i != current_last_i:
                        prev_x_coord = (float(x) + 1.0) / 10.0
                        prev_y_coord = (float(y) + 1.0) / -10.0
                        if not first_round:
                            dist = math.sqrt((prev_x_coord - prev_pivot_x)**2 + (prev_y_coord - prev_pivot_y)**2)
                            if dist > self.max_distance:
                                current_moves.append('move {} {}'.format(prev_theta, self.max_distance))
                                dists.append(self.max_distance)
                                dist -= self.max_distance
                                max_dist_reached += 1
                                while dist > self.max_distance:
                                    current_moves.append('move {} {}'.format(0, self.max_distance))
                                    dists.append(self.max_distance)
                                    dist -= self.max_distance
                                    max_dist_reached += 1
                                current_moves.append('move {} {}'.format(0, dist))
                                dists.append(dist)

                            else:
                                current_moves.append('move {} {}'.format(prev_theta, dist))
                                dists.append(dist)
                        #if turned right
                        if current_last_i == (last_i - 1) % 4:
                            theta = -1.570963
                        #if turned left
                        elif current_last_i == (last_i + 1) % 4:
                                    theta = 1.570963
                        #if it needs to turn around
                        else:
                            theta = 1.570963
                            current_moves.append('move {} {}'.format(theta, 0.0))
                        prev_theta = theta

                        if max_dist_reached != 0:
                            while max_dist_reached != 0:
                                max_dist_reached -= 1
                                angles.append(0)
                        angles.append(theta)
                        prev_pivot_x = (float(x) + 1.0) / 10.0
                        prev_pivot_y = (float(y) + 1.0) / -10.0

                    x = highest_x
                    y = highest_y
                    first_round = False
                    last_i = current_last_i

                else:
                    #if found
                    prev_x_coord = (float(final_x) + 1.0) / 10.0
                    prev_y_coord = (float(final_y) + 1.0) / -10.0
                    dist = math.sqrt((prev_x_coord - prev_pivot_x)**2 + (prev_y_coord - prev_pivot_y)**2)
                    #if the distance is greater than max, only move max then move again
                    if dist > self.max_distance:
                        current_moves.append('move {} {}'.format(prev_theta, self.max_distance))
                        dists.append(self.max_distance)
                        dist -= self.max_distance
                        while dist > self.max_distance:
                            current_moves.append('move {} {}'.format(0, self.max_distance))
                            dists.append(self.max_distance)
                            dist -= self.max_distance
                            angles.append(0)
                        current_moves.append('move {} {}'.format(0, dist))
                        dists.append(dist)
                        angles.append(0)
                    else:
                        current_moves.append('move {} {}'.format(prev_theta, dist))
                        dists.append(dist)

            #lift box
            current_moves.append('lift {}'.format(k))

            #follow path back
            current_moves.append('move {} {}'.format(1.570963, 0))
            current_dist = dists.pop()
            current_moves.append('move {} {}'.format(1.570963, current_dist))
            while len(dists) != 0:
                current_dist = dists.pop()
                current_angle = -1 * angles.pop()
                current_moves.append('move {} {}'.format(current_angle, current_dist))

            current_moves.append('down {} {}'.format(dropoff_x_coord, dropoff_y_coord))

            #move current moves to moves list
            current_moves.reverse()
            while len(current_moves) != 0:
                temp = current_moves.pop()
                moves.append(temp)


        return moves

##############################################################################
# Function to set a buffer around walls, edges, and boxes
##############################################################################

    def buffer(self, e_warehouse):
        #fill in places where walls are and add buffer of 0.2
        for i in range(len(self.warehouse)):
            for j in range(len(self.warehouse[0])):
                #if wall found, set boundry to 3
                if self.warehouse[i][j] == '#':
                    new_i = i * 10
                    new_j = j * 10

                    if new_i == 0:
                        new_i_end = 10
                    else:
                        new_i_end = new_i + 10
                        new_i = new_i - 3

                    if new_j == 0:
                        new_j_end = 10
                    else:
                        new_j_end = new_j + 10
                        new_j = new_j - 3

                    if new_i_end != len(e_warehouse):
                        new_i_end += 3

                    if new_j_end != len(e_warehouse[0]):
                        new_j_end += 3


                    for k in range(new_i, new_i_end):
                        if (new_i) >= 0 and (new_i_end) <= len(e_warehouse):
                            for l in range(new_j, new_j_end):
                                if (new_j) >= 0 and (new_j_end) <= len(e_warehouse[0]):
                                    e_warehouse[k][l] = 1
                #if dropoff found
                elif self.warehouse[i][j] == '@':
                    new_i = i * 10
                    new_j = j * 10

                    #calculate starting point
                    starting_x = float(j) + 0.5
                    starting_y = (float(i) +0.5) * -1.0

                    for k in range(new_i, (new_i + 10)):
                        for l in range(new_j, (new_j + 10)):
                            if e_warehouse[k][l] == 0:
                                e_warehouse[k][l] = 2


        #make border
        for i in range(len(e_warehouse)):
            for j in range(0, 3):
                e_warehouse[i][j] = 1
            for j in range(len(e_warehouse[i])-3, len(e_warehouse[i])):
                e_warehouse[i][j] = 1
        for i in range(len(e_warehouse[0])):
            for j in range(0, 3):
                e_warehouse[j][i] = 1
            for j in range(len(e_warehouse)-3, len(e_warehouse)):
                e_warehouse[j][i] = 1

        return starting_x, starting_y



###############################################################################
# This function creates the heuristic grid given the target x and y and the
# list delta.
##############################################################################

    def heuristic(self, target_y, target_x, e_warehouse):
        h_delta = [[-1, 0 ], # go up
                 [ 0, -1], # go left
                 [ 1, 0 ], # go down
                 [ 0, 1 ]]#, # go right

        heuristic = [[0 for col in range(len(e_warehouse[0]))] for row in range(len(e_warehouse))]
        count = 0
        y = target_y
        x = target_x

        heuristic[y][x] = count

        change = True

        while change:
            change = False
            count += 1
            for i in range(len(h_delta)):
                y2 = y + h_delta[i][0] * count
                x2 = x + h_delta[i][1] * count
                if y2 >= 0 and y2 < len(heuristic) and x2 >=0 and x2 < len(heuristic[0]):
                    heuristic[y2][x2] = count
                    change = True
                for j in range((-count), 0) + range(1, count+1):
                    y3 = y2 + h_delta[i][1] * j
                    x3 = x2 + h_delta[i][0] * j
                    if y3 >= 0 and y3 < len(heuristic) and x3 >=0 and x3 < len(heuristic[0]):
                        heuristic[y3][x3] = count
                        change = True

        return heuristic


###############################################################################
# This function creates the expansion grid given the target x, target y,
# initial x, initial y, heuristic grid, e_warehouse grid, k, and the list delta.
###############################################################################
    def expander(self, starting_y, starting_x, heuristic, target_x, target_y, delta, k, e_warehouse):
        expand = [[-1 for col in range(len(e_warehouse[0]))] for row in range(len(e_warehouse))]
        closed = [[0 for col in range(len(e_warehouse[0]))] for row in range(len(e_warehouse))]
        y = starting_y
        x = starting_x
        g = 0
        f = g + heuristic[y][x]

        open = [[f, y, x, g]]

        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand
        count = 0

        while not found and not resign:
            if len(open) == 0:
                resign = True
                print "Fail"

            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[2]
                y = next[1]
                g = next[3]
                f = next[0]
                expand[y][x] = count
                count += 1

                if x == target_x and y == target_y:
                    found = True
                else:
                    for i in range(len(delta)):
                        y2 = y + delta[i][0]
                        x2 = x + delta[i][1]
                        if y2 >= 0 and y2 < len(e_warehouse) and x2 >=0 and x2 < len(e_warehouse[0]):
                            if e_warehouse[y2][x2] == 0  or e_warehouse[y2][x2] <= (k+3) and e_warehouse[y2][x2] != 1:
                                if closed[y2][x2] == 0:
                                    #if cost is getting in the way, just delete this
                                    if i > 3:
                                        cost = 3
                                    else:
                                        cost = 2
                                    g2 = g + cost
                                    f = g2 + heuristic[y2][x2]
                                    open.append([f, y2, x2, g2])
                                    closed[y2][x2] = 1

        return expand

"""
        moves = ['move 1.570963 2.0',  # rotate and move north 2 spaces
                 'move 1.570963 0.1',  # rotate west and move closer to second box
                 'lift 1',             # lift the second box
                 'move 0.785398 1.5',  # rotate to sw and move down 1.5 squares
                 'down 3.5 -4.0',      # set the box out of the way
                 'move -0.785398 2.0', # rotate to west and move 2.5 squares
                 'move -1.570963 2.7', # rotate to north and move to pick up box 0
                 'lift 0',             # lift the first box
                 'move -1.570963 0.0', # rotate to the south east
                 'move -0.785398 1.0', # finish rotation
                 'move 0.785398 2.5',  # rotate to east and move
                 'move -1.570963 2.5', # rotate and move south
                 'down 4.5 -4.5',      # set down the box in the dropzone
                 'move -1.570963 0.6', # rotate to west and move towards box 1
                 'lift 1',             # lift the second box
                 'move 1.570963 0.0',  # rotate north
                 'move 1.570963 0.6',  # rotate east and move back towards dropzone
                 'down 4.5 -4.5']      # deliver second box

warehouse =  ['.#.',
              '.#.',
              '..@',
              '...']


warehouse =  ['#######.',
              '#.......',
              '#@......']
todo = [(7.5, -1.5),
         (7.5, -0.5)]

PI = math.pi
max_distance = 3.0
max_steering = PI / 2. + 0.01,

dp = DeliveryPlanner(warehouse, todo, max_distance, max_steering)
dp.plan_delivery()
"""
