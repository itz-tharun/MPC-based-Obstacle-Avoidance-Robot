#!/usr/bin/env python3
"""
Spawn a RED static cylinder obstacle in Gazebo.
Run this BEFORE launching the MPC node so the startup scan detects it.

Usage:
  python3 spawn_static_obstacle.py --x 1.5 --y 0.0
  python3 spawn_static_obstacle.py --x 2.0 --y 0.3 --name obs_1
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import argparse, sys

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
          <ambient>0.8 0.0 0.0 1</ambient>
          <diffuse>0.8 0.0 0.0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

def main():
    parser = argparse.ArgumentParser(
        description="Spawn a static red cylinder obstacle in Gazebo")
    parser.add_argument("--x",    type=float, default=1.5,
                        help="X position in world frame (metres)")
    parser.add_argument("--y",    type=float, default=0.0,
                        help="Y position in world frame (metres)")
    parser.add_argument("--name", type=str,   default="static_obs_0",
                        help="Unique name for this obstacle")
    args = parser.parse_args()

    rclpy.init()
    node = Node("static_obstacle_spawner")

    client = node.create_client(SpawnEntity, "/spawn_entity")
    node.get_logger().info("Waiting for /spawn_entity service...")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(
            "Gazebo /spawn_entity service not found! "
            "Is Gazebo running?")
        sys.exit(1)

    req = SpawnEntity.Request()
    req.name = args.name
    req.xml  = CYLINDER_SDF.format(name=args.name)
    req.initial_pose.position.x = args.x
    req.initial_pose.position.y = args.y
    req.initial_pose.position.z = 0.30   # half height above ground

    node.get_logger().info(
        f"Spawning '{args.name}' at x={args.x}, y={args.y} ...")
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result.success:
        node.get_logger().info(f"SUCCESS: {result.status_message}")
    else:
        node.get_logger().error(f"FAILED: {result.status_message}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
