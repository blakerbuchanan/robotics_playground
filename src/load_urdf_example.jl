using MeshCat
using RigidBodyDynamics
using MeshCatMechanisms

vis = Visualizer()
open(vis)  # open the visualizer in a separate tab/window
wait(vis)

urdf = joinpath("robot_descriptions/example_description/urdf/example.urdf")
robot = parse_urdf(urdf)

delete!(vis)
mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
set_configuration!(mvis, [0.0])