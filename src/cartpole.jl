using TrajectoryOptimization
using RobotDynamics
import RobotZoo.Cartpole
using StaticArrays, LinearAlgebra

model = Cartpole()
n,m = size(model);

N = 101
tf = 5.
dt = tf/(N-1)

x0 = @SVector zeros(n)
xf = @SVector [0, pi, 0, 0];  # i.e. swing up

# Set up
Q = 1.0e-2*Diagonal(@SVector ones(n))
Qf = 100.0*Diagonal(@SVector ones(n))
R = 1.0e-1*Diagonal(@SVector ones(m))
obj = LQRObjective(Q,R,Qf,xf,N);

# Create Empty ConstraintList
conSet = ConstraintList(n,m,N)

# Control Bounds
u_bnd = 3.0
bnd = BoundConstraint(n,m, u_min=-u_bnd, u_max=u_bnd)
add_constraint!(conSet, bnd, 1:N-1)

# Goal Constraint
goal = GoalConstraint(xf)
add_constraint!(conSet, goal, N)

# prob = Problem(model, obj, xf, tf, x0, constraints=conSet);
prob = Problem(model, obj, constraints=conSet, x0, xf, t0=0.0, tf);

u0 = @SVector fill(0.01,m)
U0 = [u0 for k = 1:N-1]
initial_controls!(prob, U0)
rollout!(prob);

# ALTRO

using Altro
opts = SolverOptions(
    cost_tolerance_intermediate=1e-2,
    penalty_scaling=10.,
    penalty_initial=1.0
)

altro = ALTROSolver(prob, opts)
set_options!(altro, show_summary=true)
solve!(altro);

println("max_violation: ", max_violation(altro))
println("cost:          ", cost(altro))
println("iterations:    ", iterations(altro));

# Extract the solution
X = states(altro)
U = controls(altro)

hcat(Vector.(X)...)

# Unconstrained iLQR

ilqr = Altro.iLQRSolver(prob, opts)
initial_controls!(ilqr, U0)
solve!(ilqr);

using Ipopt
using MathOptInterface
const MOI = MathOptInterface

# Copy problem to avoid modifying the original problem
prob_nlp = copy(prob)

# Add the dynamics and initial conditions as explicit constraints
TrajectoryOptimization.add_dynamics_constraints!(prob_nlp)

# Reset our initial guess
initial_controls!(prob_nlp, U0)
rollout!(prob_nlp)

# Create the NLP
nlp = TrajOptNLP(prob_nlp, remove_bounds=true, jac_type=:vector);

optimizer = Ipopt.Optimizer()
TrajectoryOptimization.build_MOI!(nlp, optimizer)
MOI.optimize!(optimizer)
MOI.get(optimizer, MOI.TerminationStatus())

println("max_violation: ", max_violation(nlp))
println("cost:          ", cost(nlp));

using BenchmarkTools

# Reset initial guess and then benchmark ALTRO solver
initial_controls!(altro, U0)
b_altro = benchmark_solve!(altro)

# Reset initial guess and benchmark Ipopt solver
initial_controls!(prob_nlp, U0)
rollout!(prob_nlp)
nlp = TrajOptNLP(prob_nlp, remove_bounds=true, jac_type=:vector)
Z0 = copy(TrajectoryOptimization.get_trajectory(nlp).Z)

optimizer = Ipopt.Optimizer(print_level=0)
TrajectoryOptimization.build_MOI!(nlp, optimizer)
Z = MOI.VariableIndex.(1:length(Z0))
b_ipopt = @benchmark begin
    MOI.optimize!($optimizer)
    MOI.set($optimizer, MOI.VariablePrimalStart(), $Z, $Z0)
    MOI.get($optimizer, MOI.TerminationStatus())
end

using Statistics
@show cost(nlp)
@show cost(altro)
@show max_violation(nlp)
@show max_violation(altro)
jdg = judge(median(b_altro), median(b_ipopt))
println("Speed improvement: ", round(1/ratio(jdg).time), "x")
jdg

using MeshCat
using Plots

vis = Visualizer()
render(vis)