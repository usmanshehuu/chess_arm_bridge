# Chess move planner for Gazebo Sim

This is a standalone Python tool that asks a UCI chess engine for its best move from a FEN position, validates the move, maps its source and destination squares to Cartesian poses, solves a configurable six-revolute-joint DH model, and can publish a joint trajectory through Gazebo Transport. It does not use ROS 2. The default engine is Sunfish, a Python chess engine; you can also use Stockfish or pass a move with `--move`.

The workspace had no robot model, SDF world, Gazebo installation, joint names, or calibration data when this code was written. The included JSON is a template with execution disabled. Replace its placeholders with values from your robot and simulation before attempting execution.

## Requirements

- Python 3 and `python-chess`.
- A UCI chess engine. The default is [Sunfish](https://github.com/thomasahle/sunfish), which is written in Python and provides a UCI command. Its repository documents `pip install sunfish` and the `sunfish-uci` command. Stockfish is also supported; download it from the [official Stockfish page](https://stockfishchess.org/download/) if desired.
- Gazebo **Sim** and the `gz` command line tool on `PATH` for execution.
- The arm SDF must load Gazebo Sim's `JointTrajectoryController` system for the six arm joints and configured gripper joints. The controller consumes `gz.msgs.JointTrajectory` on `/model/<model_name>/joint_trajectory` by default; configure a custom topic in both SDF and JSON if needed. See the [Gazebo Sim joint controller documentation](https://gazebosim.org/api/sim/10/jointcontrollers.html).

Install from this package directory:

```sh
python3 -m pip install -e .
```

This installs the planner dependencies and the `chess_move_gz` command. Install Sunfish in the same Python environment:

```sh
python3 -m pip install sunfish
```

A ROS 2 or `colcon` installation is not required.

## Configure your robot

Edit `config/example.json` (or copy it to a robot-specific JSON file):

- `board.a1_center_xyz_m`: center of square a1 measured in the robot base frame.
- `board.square_size_m`: center-to-center spacing between squares.
- `board.board_rpy_rad`: orientation of the board frame relative to the robot base.
- `robot.grasp_height_above_board_m` and `travel_clearance_above_board_m`: end-effector heights above the board surface.
- `kinematics.dh_standard`: six standard Denavit-Hartenberg rows `[a_m, alpha_rad, d_m, theta_offset_rad]`, in the same joint order as `execution.joint_names`.
- `kinematics.flange_to_tcp_xyz_m` and `flange_to_tcp_rpy_rad`: measured fixed transform from the final DH flange frame to the gripper TCP.
- `kinematics.lower_limits_rad`, `upper_limits_rad`, and `seed_positions_rad`: measured joint limits and a valid IK seed, in that order.
- `execution.model_name` or `joint_trajectory_topic`: the Gazebo model and actual trajectory topic.
- `execution.joint_names`: the six revolute arm joint names, matching the DH order and the SDF controller.
- `execution.gripper_joint_names` plus open/closed positions: actual gripper joint names and target positions. Use radians for revolute joints and meters for prismatic joints.
- `uci_engine.engine_path`: any UCI engine command; it defaults to `sunfish-uci`. For Stockfish, set it to the executable path.
- `execution.enabled`: leave `false` until the model, controller, joint order, calibration, IK, and collision behavior have been checked in simulation.

The numerical inverse-kinematics routine assumes a six-revolute-joint serial chain and standard DH conventions. It does not model self-collision, board/piece collisions, joint velocity limits, torque, grasp force, or actual grasp success. Gazebo's joint trajectory controller interpolates joint positions; it is not a collision-aware Cartesian motion planner. Validate the DH convention and every target in your simulator with execution disabled before considering `--execute`.

The prototype supports ordinary moves only. It recognizes and rejects captures, castling, en passant, and promotion because those need extra piece-handling actions.

## Dry run

From this package directory, after installing dependencies:

```sh
python3 -m chess_arm_bridge.gazebo_cli \
  --config config/example.json \
  --fen 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1' \
  --depth 12
```

Without `--move`, this starts the configured UCI engine, asks for its best move at the requested search depth, validates it, and prints board-frame and robot-frame Cartesian waypoint poses. It does not contact Gazebo. Pass `--engine /path/to/engine` to override the configured command, or `--movetime-ms 1000` for a time-limited search. The included example coordinates are illustrative.

## Send a trajectory to Gazebo Sim

First add the Gazebo `JointTrajectoryController` system to the arm model. Its configuration must list the six arm joints and the gripper joints in the exact order supplied in JSON, and enable position control with appropriate gains. The relevant SDF system has the form below; joint names and gains must come from your actual model and be tuned for it:

```xml
<plugin filename="gz-sim-joint-trajectory-controller-system"
        name="gz::sim::systems::JointTrajectoryController">
  <joint_name>YOUR_ARM_JOINT_1</joint_name>
  <joint_name>YOUR_ARM_JOINT_2</joint_name>
  <joint_name>YOUR_ARM_JOINT_3</joint_name>
  <joint_name>YOUR_ARM_JOINT_4</joint_name>
  <joint_name>YOUR_ARM_JOINT_5</joint_name>
  <joint_name>YOUR_ARM_JOINT_6</joint_name>
  <joint_name>YOUR_GRIPPER_JOINT</joint_name>
  <position_p_gain>GAIN_FOR_JOINT_1</position_p_gain>
  <position_p_gain>GAIN_FOR_JOINT_2</position_p_gain>
  <position_p_gain>GAIN_FOR_JOINT_3</position_p_gain>
  <position_p_gain>GAIN_FOR_JOINT_4</position_p_gain>
  <position_p_gain>GAIN_FOR_JOINT_5</position_p_gain>
  <position_p_gain>GAIN_FOR_JOINT_6</position_p_gain>
  <position_p_gain>GAIN_FOR_GRIPPER</position_p_gain>
</plugin>
```

Enable `execution.enabled` only after matching the configuration to that controller, then run:

```sh
python3 -m chess_arm_bridge.gazebo_cli \
  --config config/your_robot.json \
  --fen 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1' \
  --depth 12 --execute
```

Execution first checks that Gazebo advertises the configured topic, computes the joint trajectory, and publishes it with `gz topic`. It does not wait for controller completion or verify that the piece was grasped and placed.
