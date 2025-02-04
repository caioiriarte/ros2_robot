from urdf2mjcf import run
 
run(
    urdf_path = "/home/caioiriarte/github_ws/ros2_robot/ugv_ws/src/robot_1/urdf/robot_1.urdf",
    mjcf_path = "/home/caioiriarte/.mujoco/mujoco210/model/ugv_robot.xml",
    copy_meshes=True,
)


print(f"Archivo convertido y guardado en: {mjcf_path}")
