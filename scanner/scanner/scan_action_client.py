#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from scanner_action_interfaces.action import ScanSweep


# ── Helper progress bar ────────────────────────────────────────────────────────
def progress_bar(percent: float, width: int = 20) -> str:
    filled = int(width * percent / 100.0)
    return '█' * filled + '░' * (width - filled)


class ScanActionClient(Node):
    def __init__(self):
        super().__init__('scan_action_client')
        self._client = ActionClient(self, ScanSweep, 'scan_sweep')
        self._done   = False

    def send_goal(self):
        self.get_logger().info('Menunggu action server...')
        self._client.wait_for_server()
        self.get_logger().info('Terhubung ke server. Mengirim goal...\n')

        goal_msg         = ScanSweep.Goal()
        goal_msg.trigger = True
        goal_msg.start_position_deg = -75.0
        goal_msg.stop_position_deg = -125.0

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal DITOLAK oleh server.')
            self._done = True
            return
        self.get_logger().info('Goal DITERIMA. Sweep dimulai...\n')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb      = feedback_msg.feedback
        phase   = fb.current_phase
        pos     = fb.current_position_deg
        pct     = fb.progress_percent
        bar     = progress_bar(pct)

        phase_label = f'{phase:<12}'

        print(
            f'\r  [{phase_label}| {pct:5.1f}%] '
            f'pos={pos:+7.2f}°  {bar}',
            end='', flush=True
        )

    def _result_cb(self, future):
        print()
        result = future.result().result
        if result.success:
            self.get_logger().info(f'\nSUKSES: {result.message}')
        else:
            self.get_logger().warn(f'\nGAGAL: {result.message}')
        self._done = True


def main(args=None):
    rclpy.init(args=args)
    client = ScanActionClient()

    print('Sending GOAL to Sweep Action Server')

    client.send_goal()

    while rclpy.ok() and not client._done:
        rclpy.spin_once(client, timeout_sec=0.1)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
