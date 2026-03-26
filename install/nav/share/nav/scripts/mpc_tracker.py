#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist

from nav_msgs.msg import Path, Odometry

from sensor_msgs.msg import LaserScan

import numpy as np


OBS_DETECT_DIST = 0.5

GOAL_TOL = 0.2


class MPCTracker(Node):


    def __init__(self):

        super().__init__('mpc_tracker')


        # ✅ FIXED QoS (matches smoothing node)

        path_qos = QoSProfile(

            reliability=ReliabilityPolicy.RELIABLE,

            durability=DurabilityPolicy.TRANSIENT_LOCAL,

            history=HistoryPolicy.KEEP_LAST,

            depth=1

        )


        # ✅ FIXED topic names

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Path, '/path', self.path_cb, path_qos)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)


        self.state = np.zeros(3)

        self.path = None

        self.idx = 0

        self.odom_ok = False


        # 🔥 TUNED MPC (FAST + STABLE)

        self.N = 12

        self.dt = 0.1

        self.K = 250   # ↓ faster


        self.v_max = 0.30

        self.w_max = 2.0


        self.W_cte = 12.0

        self.W_heading = 4.0

        self.W_speed = 15.0   # 🔥 strong forward bias

        self.W_smooth = 1.0   # 🔥 less hesitation


        self.noise_v = 0.08

        self.noise_w = 0.25


        self.U_prev = np.zeros((self.N, 2))

        self.U_prev[:, 0] = 0.2


        self.last_w = 0.0


        self.create_timer(0.1, self.loop)


        self.get_logger().info("🔥 MPC READY")


    # ─────────────────────────────── #


    def path_cb(self, msg):

        self.path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

        self.idx = 0

        self.get_logger().info(f"PATH RECEIVED: {len(self.path)}")


    def odom_cb(self, msg):

        p = msg.pose.pose.position

        q = msg.pose.pose.orientation


        self.state[0] = p.x

        self.state[1] = p.y

        self.state[2] = np.arctan2(

            2*(q.w*q.z + q.x*q.y),

            1 - 2*(q.y**2 + q.z**2)

        )


        self.odom_ok = True


    def scan_cb(self, msg):

        ranges = np.array(msg.ranges)

        self.front_blocked = np.any((ranges < OBS_DETECT_DIST) & (ranges > 0.05))


    # ─────────────────────────────── #


    def get_segment(self):

        if self.path is None:

            return None


        pos = self.state[:2]


        # forward-only index update

        d = np.linalg.norm(self.path[self.idx:] - pos, axis=1)

        self.idx += int(np.argmin(d))


        end = min(self.idx + 20, len(self.path))

        return self.path[self.idx:end]


    # ─────────────────────────────── #


    def rollout(self, U):

        K = U.shape[0]


        x = np.full(K, self.state[0])

        y = np.full(K, self.state[1])

        yaw = np.full(K, self.state[2])


        traj = np.zeros((K, self.N, 3))


        for i in range(self.N):

            v = U[:, i, 0]

            w = U[:, i, 1]


            x += v * np.cos(yaw) * self.dt

            y += v * np.sin(yaw) * self.dt

            yaw += w * self.dt


            traj[:, i] = np.stack([x, y, yaw], axis=1)


        return traj


    def cost(self, U, seg):

        traj = self.rollout(U)

        cost = np.zeros(U.shape[0])


        for i in range(self.N):

            pos = traj[:, i, :2]

            yaw = traj[:, i, 2]


            diff = pos[:, None] - seg[None]

            d = np.linalg.norm(diff, axis=2)

            ni = d.argmin(axis=1)


            cost += self.W_cte * d[np.arange(len(U)), ni]**2


            nx = np.clip(ni + 1, 0, len(seg)-1)

            dx = seg[nx, 0] - pos[:, 0]

            dy = seg[nx, 1] - pos[:, 1]


            target = np.arctan2(dy, dx)

            err = np.arctan2(np.sin(target - yaw), np.cos(target - yaw))


            cost += self.W_heading * err**2


            cost -= self.W_speed * U[:, i, 0]


            if i > 0:

                cost += self.W_smooth * (

                    (U[:, i, 0] - U[:, i-1, 0])**2 +

                    (U[:, i, 1] - U[:, i-1, 1])**2

                )


        return cost


    def solve(self, seg):

        Uw = np.roll(self.U_prev, -1, axis=0)

        Uw[-1] = Uw[-2]


        nv = np.random.uniform(-self.noise_v, self.noise_v, (self.K-1, self.N))

        nw = np.random.uniform(-self.noise_w, self.noise_w, (self.K-1, self.N))


        U = np.zeros((self.K, self.N, 2))

        U[0] = Uw


        U[1:, :, 0] = np.clip(Uw[:, 0] + nv, 0.0, self.v_max)

        U[1:, :, 1] = np.clip(Uw[:, 1] + nw, -self.w_max, self.w_max)


        c = self.cost(U, seg)

        best = np.argmin(c)


        self.U_prev = U[best]


        return self.U_prev[0]


    # ─────────────────────────────── #


    def loop(self):

        if not self.odom_ok or self.path is None:

            return


        if np.linalg.norm(self.state[:2] - self.path[-1]) < GOAL_TOL:

            self.cmd_pub.publish(Twist())

            return


        seg = self.get_segment()

        if seg is None or len(seg) < 2:

            return


        v, w = self.solve(seg)


        # 🔥 prevent stopping

        v = max(v, 0.12)


        # 🔥 smooth turning (NO SWAY)

        w = 0.5*self.last_w + 0.5*w

        self.last_w = w


        cmd = Twist()

        cmd.linear.x = float(v)

        cmd.angular.z = float(w)

        self.cmd_pub.publish(cmd)


        self.get_logger().info(

            f"v={v:.2f} w={w:.2f} idx={self.idx}"

        )


# ─────────────────────────────── #


def main():

    rclpy.init()

    node = MPCTracker()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main() 
