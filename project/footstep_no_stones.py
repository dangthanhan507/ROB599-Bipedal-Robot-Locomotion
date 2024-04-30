import numpy as np
from pydrake.solvers import MathematicalProgram, Solve, GetAvailableSolvers, GetProgramType, MosekSolver, IpoptSolver
from pydrake.all import eq, ge, le

def line_intersect(slope1, b1, slope2, b2):
    x = (b2 - b1) / (slope1 - slope2)
    return x
def give_cos_boundaries():
    #first segment
    slope_l0 = -np.sin(0)
    b_l0 = np.cos(0) - slope_l0*0

    #second segment
    slope_l1 = -np.sin(np.pi/2)
    b_l1 = np.cos(np.pi/2) - slope_l1*np.pi/2

    #third segment
    slope_l2 = -np.sin(np.pi)
    b_l2 = np.cos(np.pi) - slope_l2*np.pi

    #fourth segment
    slope_l3 = -np.sin(3*np.pi/2)
    b_l3 = np.cos(3*np.pi/2) - slope_l3*3*np.pi/2

    #fifth segment
    slope_l4 = -np.sin(2*np.pi)
    b_l4 = np.cos(2*np.pi) - slope_l4*2*np.pi

    # find line intersection


    intersect01 = line_intersect(slope_l0, b_l0, slope_l1, b_l1)
    intersect12 = line_intersect(slope_l1, b_l1, slope_l2, b_l2)
    intersect23 = line_intersect(slope_l2, b_l2, slope_l3, b_l3)
    intersect34 = line_intersect(slope_l3, b_l3, slope_l4, b_l4)
    
    cos_ranges =  np.array([[0, intersect01], 
                            [intersect01, intersect12], 
                            [intersect12, intersect23], 
                            [intersect23, intersect34], 
                            [intersect34, 2*np.pi]])
    cos_slopes = np.array([slope_l0, slope_l1, slope_l2, slope_l3, slope_l4])
    cos_intercepts = np.array([b_l0, b_l1, b_l2, b_l3, b_l4])
    
    return cos_ranges, cos_slopes, cos_intercepts

def give_sin_boundaries():
    # first segment
    slope_l0 = np.cos(0)
    b_l0 = np.sin(0) - slope_l0*0
    
    # second segment
    slope_l1 = np.cos(np.pi/2)
    b_l1 = np.sin(np.pi/2) - slope_l1*np.pi/2
    
    # third segment
    slope_l2 = np.cos(np.pi)
    b_l2 = np.sin(np.pi) - slope_l2*np.pi
    
    # fourth segment
    slope_l3 = np.cos(3*np.pi/2)
    b_l3 = np.sin(3*np.pi/2) - slope_l3*3*np.pi/2
    
    # fifth segment
    slope_l4 = np.cos(2*np.pi)
    b_l4 = np.sin(2*np.pi) - slope_l4*2*np.pi
    
    # find line intersection
    intersect01 = line_intersect(slope_l0, b_l0, slope_l1, b_l1)
    intersect12 = line_intersect(slope_l1, b_l1, slope_l2, b_l2)
    intersect23 = line_intersect(slope_l2, b_l2, slope_l3, b_l3)
    intersect34 = line_intersect(slope_l3, b_l3, slope_l4, b_l4)
    
    sin_ranges =  np.array([[0, intersect01],
                            [intersect01, intersect12],
                            [intersect12, intersect23],
                            [intersect23, intersect34],
                            [intersect34, 2*np.pi]])
    sin_slopes = np.array([slope_l0, slope_l1, slope_l2, slope_l3, slope_l4])
    sin_intercepts = np.array([b_l0, b_l1, b_l2, b_l3, b_l4])
    
    return sin_ranges, sin_slopes, sin_intercepts

def footstep_planner(initial_left, initial_right, goal_pos, steps=10, step_span=0.1, reachability = 0.1, angle_step_span = 0.2):
    
    # NOTE: this will be an MISOCP
    prog = MathematicalProgram()
    
    p_left = np.array([0,  0.2, 0])
    p_right = np.array([0, 0.2, 0])
    # Variables for Feet
    left_foot_states = prog.NewContinuousVariables(rows=steps+1, cols=3, name="left_foot")
    right_foot_states = prog.NewContinuousVariables(rows=steps+1, cols=3, name="right_foot")
    
    # Integer Variables for cos/sin of footstep angle
    C = prog.NewBinaryVariables(rows=steps, cols=5, name="C")
    S = prog.NewBinaryVariables(rows=steps, cols=5, name="S")
    
    # Variables for cos/sin
    cos = prog.NewContinuousVariables(rows=steps, name="cos")
    sin = prog.NewContinuousVariables(rows=steps, name="sin")
    
    # initial conditions
    prog.AddLinearConstraint(eq(left_foot_states[0,:], initial_left))
    prog.AddLinearConstraint(eq(right_foot_states[0,:], initial_right))
    
    # step span
    # also left step every even
    # right step every odd
    for i in range(1, steps+1):
        moving_foot = left_foot_states if i % 2 == 0 else right_foot_states
        still_foot = right_foot_states if i % 2 == 0 else left_foot_states
        #step span constraints
        prog.AddLinearConstraint(moving_foot[i,0] <= moving_foot[i-1,0] + step_span)
        prog.AddLinearConstraint(moving_foot[i,1] <= moving_foot[i-1,1] + step_span)
        prog.AddLinearConstraint(moving_foot[i,2] <= moving_foot[i-1,2] + angle_step_span)
        
        prog.AddLinearConstraint(moving_foot[i,0] >= moving_foot[i-1,0] - step_span)
        prog.AddLinearConstraint(moving_foot[i,1] >= moving_foot[i-1,1] - step_span)
        prog.AddLinearConstraint(moving_foot[i,1] >= moving_foot[i-1,1] - angle_step_span)
        
        # still foot constraints
        prog.AddLinearConstraint(eq(still_foot[i,:], still_foot[i-1,:]))
        
    cos_range, cos_slope, cos_intercept = give_cos_boundaries()
    sin_range, sin_slope, sin_intercept = give_sin_boundaries()
    
    # cos/sin constraints    
    # big-M constraints to determine cos/sin based on angle
    # big-M constraints to bound angle
    M_angle = 1000
    M_assign = 1000
    for i in range(1, steps+1):
        foot = left_foot_states if i % 2 == 0 else right_foot_states
        for j in range(5):
            #bound angle 
            prog.AddLinearConstraint(foot[i,2] <= cos_range[j,1] + M_angle*(1-C[i-1,j]))
            prog.AddLinearConstraint(foot[i,2] >= cos_range[j,0] - M_angle*(1-C[i-1,j]))
            prog.AddLinearConstraint(foot[i,2] <= sin_range[j,1] + M_angle*(1-S[i-1,j]))
            prog.AddLinearConstraint(foot[i,2] >= sin_range[j,0] - M_angle*(1-S[i-1,j]))
            # assign value of cos/sin
            prog.AddLinearConstraint(cos[i-1] <= cos_slope[j]*foot[i,2] + cos_intercept[j] + M_assign*(1-C[i-1,j]))
            prog.AddLinearConstraint(cos[i-1] >= cos_slope[j]*foot[i,2] + cos_intercept[j] - M_assign*(1-C[i-1,j]))
            
            prog.AddLinearConstraint(sin[i-1] <= sin_slope[j]*foot[i,2] + sin_intercept[j] + M_assign*(1-S[i-1,j]))
            prog.AddLinearConstraint(sin[i-1] >= sin_slope[j]*foot[i,2] + sin_intercept[j] - M_assign*(1-S[i-1,j]))
    
    # ensure only pick one cos/sin range per step
    prog.AddLinearConstraint(eq(C.sum(axis=1), np.ones(steps)))
    prog.AddLinearConstraint(eq(S.sum(axis=1), np.ones(steps)))
    
    
    # reachability constraints as SOC
    # (x_j - x_j-1 - R*p)^T Q (x_j - x_j-1 - R*p) <= d
    Q = np.eye(2)
    Qhalf = np.sqrt(Q)
    for i in range(1, steps+1):
        foot = left_foot_states if i % 2 == 0 else right_foot_states
        other_foot = right_foot_states if i % 2 == 0 else left_foot_states
        # p = p_left if i % 2 == 0 else p_right
        # p_other = p_right if i % 2 == 0 else p_left
        p = p_left
        p_other = p_right
        
        R = np.array([[cos[i-1], -sin[i-1]], [sin[i-1], cos[i-1]]])
        x = foot[i,:2] - (foot[i-1,:2] + R@p[:2])
        z = np.concatenate([ np.array([reachability + 1]), 2*Qhalf @ x, np.array([reachability - 1])])
        
        # add reachability constraint for other foot
        x_other = foot[i,:2] - (other_foot[i-1,:2] + R@p_other[:2])
        z_other = np.concatenate([ np.array([reachability + 1]), 2*Qhalf @ x_other, np.array([reachability - 1])])
        
        prog.AddLorentzConeConstraint(z)
        prog.AddLorentzConeConstraint(z_other)
    
    # goal position quadratic cost as SOC
    Q = np.eye(3)
    Qhalf = np.sqrt(Q)
    slack_left = prog.NewContinuousVariables(steps+1, name="goal")
    slack_right = prog.NewContinuousVariables(steps+1, name="goal")
    for i in range(steps+1):
        zleft = np.concatenate([np.array([slack_left[i] + 1]), 2*Qhalf @ (left_foot_states[i,:] - goal_pos), np.array([slack_left[i] - 1]) ])
        prog.AddLorentzConeConstraint(zleft)
        
        zright = np.concatenate([np.array([slack_right[i] + 1]), 2*Qhalf @ (right_foot_states[i,:] - goal_pos), np.array([slack_right[i] - 1]) ])
        prog.AddLorentzConeConstraint(zright)
    
    # add cost
    prog.AddLinearCost(slack_left.sum())
    prog.AddLinearCost(slack_right.sum())
    
    # solve with Mosek
    solver = MosekSolver()
    result = solver.Solve(prog)
    
    # print solver type
    # print solver available
    print(GetProgramType(prog))
    print([solver.name() for solver in GetAvailableSolvers(GetProgramType(prog))])
    
    
    if not result.is_success():
        print("Optimization failed")
    else:
        print("Optimization successful")
    
    left_foot_sol = result.GetSolution(left_foot_states)
    right_foot_sol = result.GetSolution(right_foot_states)
    
    return left_foot_sol.T, right_foot_sol.T

if __name__ == '__main__':
    initial_left = np.array([0, -0.03, 0])
    initial_right = np.array([0, 0.03, 0])

    goal_pos = np.array([1, 0, 0])
    n_steps = 20
    step_span = 0.1
    foot_reach = 0.05

    left, right = footstep_planner(initial_left, initial_right, goal_pos, steps=n_steps, step_span=step_span, reachability=foot_reach)
    
    import matplotlib.pyplot as plt
    # plot the footstep sequence
    plt.plot(initial_left[0], initial_left[1], 'ro', label='left')
    plt.plot(initial_right[0], initial_right[1], 'bo', label='right')
    plt.plot(left[0, :], left[1, :], 'r--'
            , right[0, :], right[1, :], 'b--')

    # plot quiver
    for i in range(0, n_steps+1):
        plt.quiver(left[0, i], left[1, i], np.cos(left[2, i]), np.sin(left[2, i]), color='r')
        plt.quiver(right[0, i], right[1, i], np.cos(right[2, i]), np.sin(right[2, i]), color='b')
    plt.plot(goal_pos[0], goal_pos[1], 'go')
    plt.legend()
    plt.show()