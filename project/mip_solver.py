from project.stepping_stones import Stone, SteppingStones
from pydrake.all import MathematicalProgram, OsqpSolver, eq, ge, le, QuadraticConstraint
from pydrake.solvers import MixedIntegerBranchAndBound, GurobiSolver
import numpy as np

def set_initial_and_goal_position(prog: MathematicalProgram, stepping_stones: SteppingStones, decision_variables):
    position_left, position_right = decision_variables[:2]
    
    #initial position of feet of robot
    foot_offset = np.array([0, 0.2])
    
    #initial pos of feet
    center = stepping_stones.stones[0].center
    initial_position_left = center
    initial_position_right = center - foot_offset
    
    #goal position of feet
    center = stepping_stones.stones[-1].center
    goal_position_left = center
    goal_position_right = center - foot_offset
    
    #enforce initial position of feet
    prog.AddLinearConstraint(eq(position_left[0], initial_position_left))
    prog.AddLinearConstraint(eq(position_right[0], initial_position_right))
    
    #enforce goal position of feet
    prog.AddLinearConstraint(eq(position_left[-1], goal_position_left))
    prog.AddLinearConstraint(eq(position_right[-1], goal_position_right))

def relative_position_limits(prog, n_steps, step_span, decision_variables):
    # unpack only decision variables needed in this function
    position_left, position_right = decision_variables[:2]

    '''
        self.A = np.array([[1,0],[0,1],[-1,0],[0,-1]])
        self.b = np.concatenate([c2tr]*2) + self.A.dot(center)
    '''
    
    A = np.array([[1,0],[-1,0],[0,1],[0,-1]])
    b = np.array([step_span, step_span, step_span, step_span])
    for t in range(n_steps+1):
        left_rel_prevright = position_left[t] - position_right[t-1]
        left_rel_prevleft  = position_left[t] - position_left[t-1]
        
        right_rel_prevleft = position_right[t] - position_left[t-1]
        right_rel_prevright = position_right[t] - position_right[t-1]
        
        prog.AddLinearConstraint(le(A@left_rel_prevright, b))
        prog.AddLinearConstraint(le(A@left_rel_prevleft, b))
        prog.AddLinearConstraint(le(A@right_rel_prevleft, b))
        prog.AddLinearConstraint(le(A@right_rel_prevright, b))
    
    # for t in range(1,n_steps+2):
    #     left_rel_right = position_left[t] - position_right[t-1]
    #     right_rel_left = position_right[t] - position_left[t-1]
        
    #     prog.AddLinearConstraint(le(A@left_rel_right, b))
    #     prog.AddLinearConstraint(le(A@right_rel_left, b))
        
        
def step_sequence(prog, n_steps, step_span, decision_variables):
    position_left, position_right = decision_variables[:2]
    first_left = decision_variables[-1]
    
    first_right = 1 - first_left
    
    #limit the distance between steps
    step_limit = np.ones(2)*step_span
    
    for t in range(n_steps):
        #length of steps
        step_left = position_left[t+1] - position_left[t]
        step_right = position_right[t+1] - position_right[t]
        
        #even steps
        if t % 2 == 0:
            limit_left = step_limit * first_left
            limit_right = step_limit * first_right
        else: #odd steps
            limit_left = step_limit * first_right
            limit_right = step_limit * first_left
            
        # constraint on left-foot relative position
        # lower_limit <= step_left <= upper_limit
        prog.AddLinearConstraint(le(step_left, limit_left))
        prog.AddLinearConstraint(ge(step_left, -limit_left))
        
        # constraint on right-foot relative position
        # lower_limit <= step_right <= upper_limit
        prog.AddLinearConstraint(le(step_right, limit_right))
        prog.AddLinearConstraint(ge(step_right, -limit_right))

def one_stone_per_foot(prog, n_steps, decision_variables):
    # unpack only decision variables needed in this function
    stone_left, stone_right = decision_variables[2:4]

    # modify here
    
    # each foot is on exactly one stone at each step
    for t in range(n_steps+1):
        prog.AddLinearConstraint(eq(stone_left[t].sum(), np.ones(1)))
        prog.AddLinearConstraint(eq(stone_right[t].sum(), np.ones(1)))

def get_big_M(stepping_stones: SteppingStones):
    # big-M parameter for the horizontal axis
    initial = stepping_stones.stones[0]
    goal = stepping_stones.stones[-1]
    M = [goal.center[0] - initial.center[0]]

    # big-M parameter
    M.append(goal.center[1] - initial.center[1])
    
    return np.array(M * 2)

def foot_in_stepping_stone(prog, stepping_stones: SteppingStones, n_steps, decision_variables, M_fn=get_big_M):
    # unpack only decision variables needed in this function
    (
        position_left,
        position_right,
        stone_left,
        stone_right,
    ) = decision_variables[:4]

    # big-M vector
    M = M_fn(stepping_stones)

    n_stones = stepping_stones.num_stones()

    # modify here
    for stone in range(n_stones):
        # A_i x_t <= b_i + M_i (1 - stone_i_t)
        A = stepping_stones.stones[stone].A
        b = stepping_stones.stones[stone].b
        for t in range(n_steps+1):
            prog.AddLinearConstraint(le(A.dot(position_left[t]), b + M * (1 - stone_left[t, stone])))
            prog.AddLinearConstraint(le(A.dot(position_right[t]), b + M * (1 - stone_right[t, stone])))

def minimize_step_length(prog, n_steps, decision_variables):
    # unpack only decision variables needed in this function
    position_left, position_right = decision_variables[:2]

    # modify here
    for t in range(n_steps):
        step_length_left = position_left[t+1] - position_left[t]
        step_length_right = position_right[t+1] - position_right[t]
        
        prog.AddQuadraticCost(step_length_left.dot(step_length_left))
        prog.AddQuadraticCost(step_length_right.dot(step_length_right))

def footstep_plan(stepping_stones: SteppingStones, n_steps: int, step_span: float, M_fn=get_big_M, use_gurobi=False):
    prog = MathematicalProgram()
    n_stones = stepping_stones.num_stones()
    #make decision variables
    
    #footsteps
    position_left = prog.NewContinuousVariables(rows=n_steps+1, cols=2, name="left_foot")
    position_right = prog.NewContinuousVariables(rows=n_steps+1, cols=2, name="right_foot")
    
    # binary variables to indicate which stone the foot is on
    stone_left = prog.NewBinaryVariables(rows=n_steps+1, cols=n_stones, name="stone_left")
    stone_right = prog.NewBinaryVariables(rows=n_steps+1, cols=n_stones, name="stone_right")
    
    first_left = prog.NewBinaryVariables(1)[0]
    
    decision_variables = (position_left, position_right, stone_left, stone_right, first_left)
    
    # set boundary conditions
    set_initial_and_goal_position(prog, stepping_stones, decision_variables)
    
    # relative position limits
    relative_position_limits(prog, n_steps, step_span, decision_variables)
    
    # step sequence
    step_sequence(prog, n_steps, step_span, decision_variables)
    
    # one stone per foot
    one_stone_per_foot(prog, n_steps, decision_variables)
    
    # foot in stepping stone
    foot_in_stepping_stone(prog, stepping_stones, n_steps, decision_variables, M_fn=M_fn)
    
    # minimize step length
    minimize_step_length(prog, n_steps, decision_variables)
    
    if use_gurobi:
        bb = MixedIntegerBranchAndBound(prog, GurobiSolver().solver_id())
    else:
        bb = MixedIntegerBranchAndBound(prog, OsqpSolver().solver_id())
    result = bb.Solve()
    if result != result.kSolutionFound:
        raise ValueError("Infeasible optimization problem.")
    
    decision_variables_opt = [bb.GetSolution(var) for var in decision_variables]
    return decision_variables_opt