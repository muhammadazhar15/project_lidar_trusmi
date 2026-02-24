#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2, Imu

from scanner_action_interfaces.action import ScanSweep
from scanner_assembler.srv import AssemblerQuery


# ── Konstanta gerakan ──────────────────────────────────────────────────────────
INTERVAL_MS      = 100
INTERVAL_S       = INTERVAL_MS / 1000.0

DEG_FAST         = 4.0
DEG_SLOW         = 0.4

POS_START_DEG    = 0.0
REACH_TOLERANCE  = 0.5

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi


class ScanActionServer(Node):
    def __init__(self):
        super().__init__('scan_action_server')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self._cb_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self._assembled_pub = self.create_publisher(PointCloud2, '/assembled_cloud', 10)

        self._current_pos_rad = 0.0 # default value
        self.pos_start_deg = -75 # default value
        self.pos_stop_deg = -125 # default value

        self._imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_cb,
            10,
            callback_group=self._cb_group
        )

        # ── Assembler service client ───────────────────────────────────────────
        self._assembler_client = self.create_client(
            AssemblerQuery,
            '/assembler_service',
            callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self,
            ScanSweep,
            'scan_sweep',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group
        )

        self.get_logger().info('ScanActionServer siap. Menunggu goal...')

    def _goal_cb(self, goal_request):
        if not goal_request.trigger:
            return GoalResponse.REJECT
        self.pos_start_deg      = goal_request.start_position_deg
        self.pos_stop_deg      = goal_request.stop_position_deg
        self.get_logger().info('Goal diterima — memulai sweep.')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel diminta.')
        return CancelResponse.ACCEPT

    def _imu_cb(self, msg: Imu):
        try:
            # Quaternion
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            quaternion = (qx, qy, qz, qw)
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            # The default order for Euler angle rotations in this library is ZYX (fixed frame RPY)
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            self._current_pos_rad = roll

        except (ValueError, IndexError):
            pass

    def _print_header(self):
        print()
        print('─' * 75)
        print(f"{'Step':>5} │ {'Fase':<12} │ {'Setpoint':>10} │ {'Aktual GZ':>10} │ {'Error':>8} │ {'Progress':>8}")
        print('─' * 75)

    def _print_row(self, step, phase, setpoint_deg, actual_deg, progress):
        error = actual_deg - setpoint_deg
        phase_label = f'{phase:<12}'
        bar_width = 10
        filled = int(bar_width * progress / 100.0)
        bar = '█' * filled + '░' * (bar_width - filled)
        print(
            f'{step:>5} │ {phase_label} │ {setpoint_deg:>+9.2f}° │ '
            f'{actual_deg:>+9.2f}° │ {error:>+7.2f}° │ {progress:>5.1f}% {bar}'
        )

    def _request_assembled_cloud(self, begin_time, end_time):
        """Request pointcloud gabungan dari assembler berdasarkan time range."""
        if not self._assembler_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Assembler service tidak tersedia!')
            return None

        req = AssemblerQuery.Request()
        req.begin = begin_time
        req.end   = end_time
        req.clear_data = False

        future = self._assembler_client.call_async(req)

        # Tunggu future selesai tanpa blocking executor
        timeout = 5.0
        start = time.time()
        while not future.done():
            elapsed = time.time() - start
            if elapsed > timeout:
                self.get_logger().error('Timeout menunggu assembler!')
                return None
            time.sleep(0.05)

        if future.result() is not None:
            cloud = future.result().cloud
            self.get_logger().info(
                f'Assembled cloud diterima! '
                f'Jumlah titik: {cloud.width * cloud.height}'
            )
            self._assembled_pub.publish(cloud)
            return cloud
        else:
            self.get_logger().error('Gagal mendapat response dari assembler!')
            return None

    def _execute_cb(self, goal_handle):
        self.get_logger().info('Eksekusi sweep dimulai.')
        feedback_msg = ScanSweep.Feedback()

        steps_fast_go  = int(abs(self.pos_start_deg - POS_START_DEG) / DEG_FAST)
        steps_sweeping = int(abs(self.pos_stop_deg - self.pos_start_deg)   / DEG_SLOW)
        steps_fast_ret = int(abs(POS_START_DEG - self.pos_stop_deg) / DEG_FAST)
        total_steps    = steps_fast_go + steps_sweeping + steps_fast_ret
        step_count     = 0

        rate = self.create_rate(1.0 / INTERVAL_S)

        segments = [
            {
                'phase'    : 'fast_go',
                'target'   : self.pos_start_deg,
                'step_deg' : DEG_FAST,
            },
            {
                'phase'    : 'sweeping',
                'target'   : self.pos_stop_deg,
                'step_deg' : DEG_SLOW,
            },
            {
                'phase'    : 'fast_return',
                'target'   : POS_START_DEG,
                'step_deg' : DEG_FAST,
            },
        ]

        setpoint_deg     = rad2deg(self._current_pos_rad)
        sweep_begin_time = None
        sweep_end_time   = None

        self._print_header()
        
        sweep_begin_tmp = None
        sweep_end_tmp   = None
        req_tmp = AssemblerQuery.Request()
        req_tmp.clear_data = True

        for seg in segments:
            phase     = seg['phase']
            target    = seg['target']
            step_deg  = seg['step_deg']

            print(f"  ── [{phase.upper()}] target={target:+.1f}° step={step_deg}°/100ms ──")

            # Catat waktu mulai fase sweeping
            if phase == 'sweeping':
                sweep_begin_time = self.get_clock().now().to_msg()
                self.get_logger().info(
                    f'Clock saat sweep mulai: sec={sweep_begin_time.sec} nanosec={sweep_begin_time.nanosec}')
                self.get_logger().info('Assembler: mulai kumpulkan pointcloud...')
                sweep_begin_tmp = sweep_begin_time
                req_tmp.begin = sweep_begin_tmp

            while True:
                if goal_handle.is_cancel_requested:
                    print('─' * 75)
                    goal_handle.canceled()
                    result = ScanSweep.Result()
                    result.success = False
                    result.message = 'Sweep dibatalkan.'
                    self._publish_cmd(setpoint_deg)
                    return result

                remaining = abs(target - setpoint_deg)
                if remaining <= REACH_TOLERANCE:
                    setpoint_deg = target
                    self._publish_cmd(setpoint_deg)
                    break
                
                if target >= setpoint_deg:
                    setpoint_deg += step_deg
                    setpoint_deg = min(setpoint_deg, target)
                elif target < setpoint_deg:
                    setpoint_deg -= step_deg
                    setpoint_deg = max(setpoint_deg, target)

                self._publish_cmd(setpoint_deg)
                step_count += 1

                actual_deg = rad2deg(self._current_pos_rad)
                progress   = min(100.0, (step_count / (total_steps)) * 100.0)

                self._print_row(step_count, phase, setpoint_deg, actual_deg, progress)

                if phase == 'sweeping':
                    sweep_end_tmp = self.get_clock().now().to_msg()
                    req_tmp.end   = sweep_end_tmp
                    assembly_tmp = self._assembler_client.call_async(req_tmp)
                    while not assembly_tmp.done():
                        self.get_logger().info("Wait Response")
                    if assembly_tmp.result() is not None:
                        cloud_tmp = assembly_tmp.result().cloud
                        self._assembled_pub.publish(cloud_tmp)

                feedback_msg.current_position_deg = round(actual_deg, 2)
                feedback_msg.current_phase        = phase
                feedback_msg.progress_percent     = round(progress, 1)
                goal_handle.publish_feedback(feedback_msg)
                
                rate.sleep()

            if phase == 'sweeping':
                sweep_end_time = self.get_clock().now().to_msg()
                self.get_logger().info(
                    f'Clock saat sweep selesai: sec={sweep_end_time.sec} nanosec={sweep_end_time.nanosec}')
                self.get_logger().info('Assembler: fase sweeping selesai, meminta pointcloud...')

        print('─' * 75)
        print(f"  Posisi akhir aktual dari Gazebo: {rad2deg(self._current_pos_rad):+.2f}°")
        print('─' * 75)

        # ── Request assembled pointcloud ───────────────────────────────────────
        if sweep_begin_time and sweep_end_time:
            assembled_cloud = self._request_assembled_cloud(sweep_begin_time, sweep_end_time)
            if assembled_cloud:
                point_count = assembled_cloud.width * assembled_cloud.height
                print(f"  Rekonstruksi 3D selesai! Total titik: {point_count}")
            else:
                print('  Rekonstruksi 3D gagal.')

        result = ScanSweep.Result()
        result.success = True
        result.message = (
            f'Sweep selesai! Total langkah: {step_count}. '
            f'Posisi akhir: {rad2deg(self._current_pos_rad):.1f}°'
        )
        self.get_logger().info("MISI SELESAI!!")
        goal_handle.succeed()
        return result

    def _publish_cmd(self, deg: float):
        msg = Float64MultiArray()
        msg.data = [deg2rad(deg)]
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    server = ScanActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
