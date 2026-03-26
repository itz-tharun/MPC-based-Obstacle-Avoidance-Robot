#!/usr/bin/env python3
"""
Spawn an ORANGE dynamic cylinder obstacle in Gazebo mid-run.
Run this in a NEW terminal while the robot is already moving.

Usage:
  # Drop obstacle in front of robot, keep it there
  python3 spawn_dynamic_obstacle.py --x 2.0 --y 0.0

  # Drop obstacle and remove it after 8 seconds (robot resumes)
  python3 spawn_dynamic_obstacle.py --x 2.0 --y 0.0 --remove-after 8

  # Multiple obstacles
  python3 spawn_dynamic_obstacle.py --x 1.5 --y 0.0  --name dyn_0
  python3 spawn_dynamic_obstacle.py --x 2.5 --y 0.2  --name dyn_1
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import argparse, sys, time

CYLINDER_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.18</radius>
            <length>0.60</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.18</radius>
            <length>0.60</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.5 0.0 1</ambient>
          <diffuse>1.0 0.5 0.0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

def main():
    parser = argparse.ArgumentParser(
        description="Spawn a dynamic orange cylinder obstacle mid-run")
    parser.add_argument("--x",            type=float, default=2.0)
    parser.add_argument("--y",            type=float, default=0.0)
    parser.add_argument("--name",         type=str,   default="dyn_obs_0")
    parser.add_argument("--remove-after", type=float, default=0.0,
        help="Auto-remove after N seconds. 0 = keep forever.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("dynamic_obstacle_spawner")

    # ── spawn ── #
    spawn_cli = node.create_client(SpawnEntity, "/spawn_entity")
    node.get_logger().info("Waiting for /spawn_entity ...")
    if not spawn_cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("Gazebo not running!")
        sys.exit(1)

    req      = SpawnEntity.Request()
    req.name = args.name
    req.xml  = CYLINDER_SDF.format(name=args.name)
    req.initial_pose.position.x = args.x
    req.initial_pose.position.y = args.y
    req.initial_pose.position.z = 0.30

    future = spawn_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result.success:
        node.get_logger().info(
            f"Spawned '{args.name}' at ({args.x},{args.y}) — "
            f"{result.status_message}")
    else:
        node.get_logger().error(f"Spawn failed: {result.status_message}")
        sys.exit(1)

    # ── optional auto-remove ── #
    if args.remove_after > 0:
        node.get_logger().info(
            f"Will remove in {args.remove_after:.1f} seconds...")
        time.sleep(args.remove_after)

        del_cli = node.create_client(DeleteEntity, "/delete_entity")
        del_cli.wait_for_service(timeout_sec=5.0)
        del_req      = DeleteEntity.Request()
        del_req.name = args.name
        del_future   = del_cli.call_async(del_req)
        rclpy.spin_until_future_complete(node, del_future)
        node.get_logger().info(f"Removed '{args.name}'")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
