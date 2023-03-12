using MeshCat
using RigidBodyDynamics
using MeshCatMechanisms

vis = Visualizer()
open(vis)  # open the visualizer in a separate tab/window
wait(vis)

urdf = joinpath(dirname(pathof(MeshCatMechanisms)), "..", "test", "urdf", "Acrobot.urdf")
robot = parse_urdf(urdf)

delete!(vis)
mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
set_configuration!(mvis, [0.0, 0.0])

state = MechanismState(robot, randn(2), randn(2))
t, q, v = simulate(state, 5.0);

animation = Animation(mvis, t, q)
setanimation!(mvis, animation)