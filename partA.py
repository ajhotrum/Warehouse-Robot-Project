'''
=== Introduction ===

In this problem, you will build a planner that helps a robot
  find the best path through a warehouse filled with boxes
  that it has to pick up and deliver to a dropzone.

Your file must be called `partA.py` and must have a class
  called `DeliveryPlanner`.
This class must have an `__init__` function that takes three
  arguments: `self`, `warehouse`, and `todo`.
The class must also have a function called `plan_delivery` that
  takes a single argument, `self`.

=== Input Specifications ===

`warehouse` will be a list of m strings, each with n characters,
  corresponding to the layout of the warehouse. The warehouse is an
  m x n grid. warehouse[i][j] corresponds to the spot in the ith row
  and jth column of the warehouse, where the 0th row is the northern
  end of the warehouse and the 0th column is the western end.

The characters in each string will be one of the following:

'.' (period) : traversable space. The robot may enter from any adjacent space.
'#' (hash) : a wall. The robot cannot enter this space.
'@' (dropzone): the starting point for the robot and the space where all boxes must be delivered.
  The dropzone may be traversed like a '.' space.
[0-9a-zA-Z] (any alphanumeric character) : a box. At most one of each alphanumeric character
  will be present in the warehouse (meaning there will be at most 62 boxes). A box may not
  be traversed, but if the robot is adjacent to the box, the robot can pick up the box.
  Once the box has been removed, the space functions as a '.' space.

For example,
  warehouse = ['1#2',
               '.#.',
               '..@']
  is a 3x3 warehouse.
  - The dropzone is at the warehouse cell in row 2, column 2.
  - Box '1' is located in the warehouse cell in row 0, column 0.
  - Box '2' is located in the warehouse cell in row 0, column 2.
  - There are walls in the warehouse cells in row 0, column 1 and row 1, column 1.
  - The remaining five warehouse cells contain empty space. (The dropzone is empty space)
#
The argument `todo` is a list of alphanumeric characters giving the order in which the
  boxes must be delivered to the dropzone. For example, if
  todo = ['1','2']
  is given with the above example `warehouse`, then the robot must first deliver box '1'
  to the dropzone, and then the robot must deliver box '2' to the dropzone.

=== Rules for Movement ===

- Two spaces are considered adjacent if they share an edge or a corner.
- The robot may move horizontally or vertically at a cost of 2 per move.
- The robot may move diagonally at a cost of 3 per move.
- The robot may not move outside the warehouse.
- The warehouse does not "wrap" around.
- As described earlier, the robot may pick up a box that is in an adjacent square.
- The cost to pick up a box is 4, regardless of the direction the box is relative to the robot.
- While holding a box, the robot may not pick up another box.
- The robot may put a box down on an adjacent empty space ('.') or the dropzone ('@') at a cost
  of 2 (regardless of the direction in which the robot puts down the box).
- If a box is placed on the '@' space, it is considered delivered and is removed from the ware-
  house.
- The warehouse will be arranged so that it is always possible for the robot to move to the
  next box on the todo list without having to rearrange any other boxes.

An illegal move will incur a cost of 100, and the robot will not move (the standard costs for a
  move will not be additionally incurred). Illegal moves include:
- attempting to move to a nonadjacent, nonexistent, or occupied space
- attempting to pick up a nonadjacent or nonexistent box
- attempting to pick up a box while holding one already
- attempting to put down a box on a nonadjacent, nonexistent, or occupied space
- attempting to put down a box while not holding one

=== Output Specifications ===

`plan_delivery` should return a LIST of moves that minimizes the total cost of completing
  the task successfully.
Each move should be a string formatted as follows:

'move {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot moves
  to and '{j}' is replaced by the column-coordinate of the space the robot moves to

'lift {x}', where '{x}' is replaced by the alphanumeric character of the box being picked up

'down {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot puts
  the box, and '{j}' is replaced by the column-coordinate of the space the robot puts the box

For example, for the values of `warehouse` and `todo` given previously (reproduced below):
  warehouse = ['1#2',
               '.#.',
               '..@']
  todo = ['1','2']
`plan_delivery` might return the following:
  ['move 2 1',
   'move 1 0',
   'lift 1',
   'move 2 1',
   'down 2 2',
   'move 1 2',
   'lift 2',
   'down 2 2']

=== Grading ===

- Your planner will be graded against a set of test cases, each equally weighted.
- If your planner returns a list of moves of total cost that is K times the minimum cost of
  successfully completing the task, you will receive 1/K of the credit for that test case.
- Otherwise, you will receive no credit for that test case. This could happen for one of several
  reasons including (but not necessarily limited to):
  - plan_delivery's moves do not deliver the boxes in the correct order.
  - plan_delivery's output is not a list of strings in the prescribed format.
  - plan_delivery does not return an output within the prescribed time limit.
  - Your code raises an exception.

=== Additional Info ===

- You may add additional classes and functions as needed provided they are all in the file `partA.py`.
- Upload partA.py to  Canvas in the Assignments section. Do not put it into an
  archive with other files.
- Your partA.py file must not execute any code when imported.
- Ask any questions about the directions or specifications on Piazza.
'''


class DeliveryPlanner:

    #initialize a warehouse and todo lists
    warehouse = []
    todo = []

    def __init__(self, warehouse, todo):
        pass
        self.warehouse = warehouse
        self.todo = todo


###############################################################################
# This function creates the heuristic grid given the target x and y and the
# list delta.
##############################################################################

    def heuristic(self, target_y, target_x, delta):
        #create heuristic and closed 2D arrays
        heuristic = [[" " for col in range(len(self.warehouse[0]))] for row in range(len(self.warehouse))]
        closed_h = [[0 for col in range(len(self.warehouse[0]))] for row in range(len(self.warehouse))]

        #set current x and y to the targets
        x = target_x
        y = target_y

        #initial heuristic value is 0. Close this node in the closed array
        heuristic[y][x] = 0
        closed_h[y][x] = 1
        count_h = 0

        #set current to the first item in open
        open_h = [[count_h, y, x]]

        #while there are still options in the arrays
        while len(open_h) != 0:
            #pop from the item with the smallest count
            open_h.sort()
            open_h.reverse()
            next_h = open_h.pop()
            count_h = next_h[0]
            y = next_h[1]
            x = next_h[2]

            #set the values around the current equal to the current + 1
            for i in range(len(delta)):
                y2 = y + delta[i][0]
                x2 = x + delta[i][1]
                if y2 >= 0 and y2 < len(self.warehouse) and x2 >=0 and x2 < len(self.warehouse[0]):
                    if heuristic[y2][x2] > heuristic[y][x] and closed_h[y][x] != 0:
                      heuristic[y2][x2] = heuristic[y][x] + 1
                      closed_h[y2][x2] = 1
                      count_h = count_h +1
                      #append to open
                      open_h.append([count_h, y2, x2])

        return heuristic


###############################################################################
# This function creates the expansion grid given the target x, target y,
# initial x, initial y, heuristic grid, allowable grid, k, and the list delta.
##############################################################################

    def expander(self, initial_y, initial_x, heuristic, target_x, target_y, delta, k, allowable):

        #initialize expand and closed 2D arrays
        expand = [[-1 for col in range(len(self.warehouse[0]))] for row in range(len(self.warehouse))]
        closed = [[0 for col in range(len(self.warehouse[0]))] for row in range(len(self.warehouse))]
        y = initial_y
        x = initial_x
        g = 0
        f = g + heuristic[y][x]

        #set the initial cell as the first open in the list
        open = [[f, y, x, g]]

        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand
        count = 0

        while not found and not resign:
            if len(open) == 0:
                #no nodes left to check. Failure.
                resign = True
                print "Fail"

            else:
                open.sort()
                open.reverse()
                next = open.pop()

                #set to current values of lowest f
                x = next[2]
                y = next[1]
                g = next[3]
                f = next[0]
                expand[y][x] = count
                count += 1

                if x == target_x and y == target_y:
                    found = True
                else:
                    #check in every direction
                    for i in range(len(delta)):
                        y2 = y + delta[i][0]
                        x2 = x + delta[i][1]
                        #if within array and allawable and not closed or if the box is found
                        if y2 >= 0 and y2 < len(self.warehouse) and x2 >=0 and x2 < len(self.warehouse[0]):
                            if allowable[y2][x2] == 0  or self.warehouse[y2][x2] == self.todo[k]:
                                if closed[y2][x2] == 0:
                                    #add costs associated with moves
                                    if i > 3:
                                        cost = 3
                                    else:
                                        cost = 2

                                    #update values and append
                                    g2 = g + cost
                                    f = g2 + heuristic[y2][x2]
                                    open.append([f, y2, x2, g2])

                                    #close in the closed array
                                    closed[y2][x2] = 1

        return expand


    def plan_delivery(self):

        #list of movement directions
        delta = [[-1, 0 ], # go up
                 [ 0, -1], # go left
                 [ 1, 0 ], # go down
                 [ 0, 1 ], # go right
                 [ 1, 1 ], # go down and right
                 [-1, -1], # go up and left
                 [-1, 1 ], # go up and right
                 [ 1, -1]] # go up and left

        moves = []
        first_pass = True

        #find starting point and set starting and dropoff values
        for i in range(len(self.warehouse)):
            for j in range(len(self.warehouse[0])):
                if self.warehouse[i][j] == '@':
                    dropoff_x = j
                    dropoff_y = i
                    initial_x = j
                    initial_y = i


        #set up allowable grid
        allowable = [[1 for col in range(len(self.warehouse[0]))] for row in range(len(self.warehouse))]
        for i in range(len(self.warehouse)):
            for j in range(len(self.warehouse[0])):
                if self.warehouse[i][j] == '.' or self.warehouse[i][j] == '@':
                    allowable[i][j] = 0

        #start loop for all items
        for k in range(len(self.todo)):
            #find target position for current target
            for i in range(len(self.warehouse)):
                for j in range(len(self.warehouse[0])):
                  if self.warehouse[i][j] == self.todo[k]:
                    target_x = j
                    target_y = i

            allowable[target_y][target_x] = 0
            switcharoo = False

            #check if the goal is right next to the starting point
            if first_pass:
                for i in range(len(delta)):
                        y2 = dropoff_y + delta[i][0]
                        x2 = dropoff_x + delta[i][1]
                        if x2 == target_x and y2 == target_y:
                            moves.append('lift {}'.format(self.todo[k]))
                            moves.append('move {} {}'.format(y2, x2))
                            moves.append('down {} {}'.format(dropoff_y, dropoff_x))
                            initial_x = x2
                            initial_y = y2
                            switcharoo = True

            #if the box is not right next to the starting point
            if not switcharoo:
                #heuristic grid
                heuristic = self.heuristic(target_y, target_x, delta)


                #expansion grid
                expand_grid = self.expander(initial_y, initial_x, heuristic, target_x, target_y, delta, k, allowable)


                current_moves = []
                current_moves.append('lift {}'.format(self.todo[k]))

                #check around
                x = target_x
                y = target_y

                #values to hold the x and y value of the lowest surrounding block
                lowest_x = 0
                lowest_y = 0

                found = False
                first_round = True
                closed = [[0 for col in range(len(self.warehouse[0]))] for row in range(len(self.warehouse))]
                while not found:
                    current_low = 99999
                    for i in range(len(delta)):
                        y2 = y + delta[i][0]
                        x2 = x + delta[i][1]
                        #check if you're back at starting point
                        if x2 == initial_x and y2 == initial_y:
                            found = True
                        else:
                            if y2 >= 0 and y2 < len(self.warehouse) and x2 >=0 and x2 < len(self.warehouse[0]):
                                if closed[y2][x2] == 0:
                                    #expand to the lowest cell around
                                    if expand_grid[y2][x2] >= 0 and expand_grid[y2][x2] < current_low:
                                        current_low = expand_grid[y2][x2]
                                        lowest_x = x2
                                        lowest_y = y2
                                        last_x = x
                                        last_y = y
                                        nothing_around = False
                    if not found:
                        if nothing_around:
                            #go back to where you last were
                            closed[y][x] = 1
                            x = last_x
                            y = last_y
                        else:
                            #update to new x and y
                            closed[y][x] = 1
                            x = lowest_x
                            y = lowest_y
                            #append the move to the current moves list
                            current_moves.append('move {} {}'.format(y, x))
                            if first_round:
                                x_before_goal = x
                                y_before_goal = y
                                first_round = False

                if first_round:
                    #swap
                    x_before_goal = initial_x
                    y_before_goal = initial_y
                    first_round = False

                for i in range(len(current_moves)):
                    temp = current_moves.pop()
                    moves.append(temp)


                #update the initial x and y to the ones before the goal
                initial_x = x_before_goal
                initial_y = y_before_goal

                #calculate heuristic
                heuristic = self.heuristic(dropoff_y, dropoff_x, delta)

                #make current moves list as a place holder
                current_moves = []
                current_moves.append('down {} {}'.format(dropoff_y, dropoff_x))


                #calculate the expand grid
                expand_grid = self.expander(y_before_goal, x_before_goal, heuristic, dropoff_x, dropoff_y, delta, k, allowable)


                #check around
                x = dropoff_x
                y = dropoff_y


                #values to hold the x and y value of the lowest surrounding block
                lowest_x = 0
                lowest_y = 0
                found = False
                first_round = True

                #reset the closed array
                closed = [[0 for col in range(len(self.warehouse[0]))] for row in range(len(self.warehouse))]

                #choose the path
                while not found:
                    current_low = 99999
                    nothing_around = True
                    for i in range(len(delta)):
                        y2 = y + delta[i][0]
                        x2 = x + delta[i][1]
                        #check if you're back at starting point
                        if x2 == x_before_goal and y2 == y_before_goal:
                            found = True
                        else:
                            if y2 >= 0 and y2 < len(self.warehouse) and x2 >=0 and x2 < len(self.warehouse[0]):
                                #check for the lowest number and expand there
                                if closed[y2][x2] == 0:
                                    if expand_grid[y2][x2] >= 0 and expand_grid[y2][x2] < current_low:
                                        current_low = expand_grid[y2][x2]
                                        lowest_x = x2
                                        lowest_y = y2
                                        last_x = x
                                        last_y = y
                                        nothing_around = False
                    if not found:
                        if nothing_around:
                            #go back to where you last were
                            closed[y][x] = 1
                            x = last_x
                            y = last_y
                        else:
                            #set new x and y
                            closed[y][x] = 1
                            x = lowest_x
                            y = lowest_y
                            #append the move to the current moves list
                            current_moves.append('move {} {}'.format(y, x))
                            if first_round:
                                x_before_drop = x
                                y_before_drop = y
                                first_round = False

                #label the x and y before drop
                if first_round:
                    x_before_drop = x
                    y_before_drop = y
                    first_round = False

                #pop the current moves into moves
                for i in range(len(current_moves)):
                    temp = current_moves.pop()
                    moves.append(temp)


                #set initial x and y to the x and y before the drop
                initial_x = x_before_drop
                initial_y = y_before_drop




            #no longer the first pass
            first_pass = False

        return moves
