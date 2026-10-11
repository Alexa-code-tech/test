#!/usr/bin/env python3
"""Миссия A->B: NavSatFix -> локальные ENU-координаты (frame map) -> Nav2 NavigateToPose."""
import math
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformListener


def gps_to_enu(lat, lon, lat0, lon0):
    """Метры на градус для WGS84 около lat0; возвращает (east, north)."""
    phi = math.radians(lat0)
    m_lat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    m_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return (lon - lon0) * m_lon, (lat - lat0) * m_lat


class GpsWaypointNav(Node):
    def __init__(self):
        super().__init__('gps_waypoint_nav')
        self.declare_parameter('waypoints_file', '')
        self.fixes = []
        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, 10)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.state_cli = self.create_client(GetState, '/bt_navigator/get_state')

    def on_fix(self, msg):
        if len(self.fixes) < 10 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.fixes.append((msg.latitude, msg.longitude))

    def wait(self, cond, what, timeout=180.0):
        self.get_logger().info(f'Жду: {what}')
        t0 = time.time()
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > timeout:
                raise RuntimeError(f'Таймаут ожидания: {what}')

    def nav2_active(self):
        if not self.state_cli.service_is_ready():
            return False
        fut = self.state_cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        return fut.done() and fut.result().current_state.id == State.PRIMARY_STATE_ACTIVE

    def go(self, x, y, yaw, name):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        def fb(m):
            self.get_logger().info(
                f'[{name}] осталось {m.feedback.distance_remaining:.1f} м', throttle_duration_sec=5.0)

        fut = self.nav.send_goal_async(goal, feedback_callback=fb)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().warn(f'[{name}] цель отклонена')
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            return False
        t = self.tf_buf.lookup_transform('map', 'base_link', Time()).transform.translation
        err = math.hypot(t.x - x, t.y - y)
        if err > 0.5:
            self.get_logger().error(f'[{name}] Nav2 сообщил об успехе, но до цели {err:.1f} м')
            return False
        return True

    def run(self):
        with open(self.get_parameter('waypoints_file').value) as f:
            wps = yaml.safe_load(f)['waypoints']

        self.wait(lambda: len(self.fixes) >= 5, 'GPS fix (/gps/fix)')
        lat0 = sum(f[0] for f in self.fixes) / len(self.fixes)
        lon0 = sum(f[1] for f in self.fixes) / len(self.fixes)
        self.get_logger().info(f'Старт по GPS: {lat0:.7f}, {lon0:.7f} (начало frame map)')

        self.wait(lambda: self.tf_buf.can_transform('map', 'base_link', Time()), 'TF map->base_link (SLAM)')
        self.wait(self.nav2_active, 'Nav2 (bt_navigator active)')
        self.nav.wait_for_server()

        prev = (0.0, 0.0)
        for w in wps:
            x, y = gps_to_enu(w['lat'], w['lon'], lat0, lon0)
            yaw = math.atan2(y - prev[1], x - prev[0])
            self.get_logger().info(f"Еду к {w['name']}: map=({x:.2f}, {y:.2f})")
            ok = False
            for attempt in range(3):
                ok = self.go(x, y, yaw, w['name'])
                if ok:
                    break
                self.get_logger().warn(f"{w['name']}: попытка {attempt + 1} не удалась")
            if not ok:
                self.get_logger().error(f"Не удалось доехать до {w['name']}, миссия прервана")
                return False
            self.get_logger().info(f"{w['name']} достигнута")
            prev = (x, y)

        t = self.tf_buf.lookup_transform('map', 'base_link', Time()).transform.translation
        self.get_logger().info(
            f'МИССИЯ ВЫПОЛНЕНА. Ошибка до B: {math.hypot(t.x - prev[0], t.y - prev[1]):.2f} м')
        return True


def main():
    rclpy.init()
    node = GpsWaypointNav()
    try:
        node.run()
    except (KeyboardInterrupt, RuntimeError) as e:
        node.get_logger().error(str(e))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
