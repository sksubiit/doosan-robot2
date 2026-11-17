import time
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from dsr_robotiq2f.driver import FAULTS, OBJ, Robotiq2F


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper')
        d = self.declare_parameter
        self.port = d('port', '/dev/ttyUSB0').value
        self.slave = d('slave_id', 9).value
        self.baud = d('baudrate', 115200).value
        self.rate = d('poll_rate_hz', 50.0).value
        self.joint = d('joint_name', 'robotiq_85_left_knuckle_joint').value
        self.max_open = d('max_opening', 0.085).value
        self.raw_open = d('raw_open', 3).value
        self.raw_closed = d('raw_closed', 228).value
        self.def_speed = d('default_speed', 100).value
        self.def_force = d('default_force', 60).value
        self.max_speed = d('max_speed', 255).value
        self.max_force = d('max_force', 150).value
        self.max_effort_n = d('max_effort_n', 235.0).value
        auto = d('auto_activate', False).value

        self.drv = Robotiq2F(self.port, self.slave, self.baud)
        if not self.drv.connect():
            raise RuntimeError(f'cannot open {self.port} (dialout group? port in use?)')
        self.status = None
        self._comm_ok = True
        self._activating = threading.Event()

        poll_cg = MutuallyExclusiveCallbackGroup()
        self.srv_cg = ReentrantCallbackGroup()
        self.pub_js = self.create_publisher(JointState, 'gripper/joint_states', 10)
        self.pub_diag = self.create_publisher(DiagnosticStatus, 'gripper/status', 10)
        self.create_timer(1.0 / self.rate, self._poll, callback_group=poll_cg)
        self.create_service(Trigger, 'gripper/activate', self._srv_activate, callback_group=self.srv_cg)
        self.create_service(Trigger, 'gripper/reset', self._srv_reset, callback_group=self.srv_cg)
        self.create_service(Trigger, 'gripper/stop', self._srv_stop, callback_group=self.srv_cg)
        self._action = ActionServer(
            self, GripperCommand, 'gripper/gripper_cmd', self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=self.srv_cg)
        self.get_logger().info(f'Robotiq 2F on {self.port} slave {self.slave} @ {self.rate} Hz')
        if auto:
            threading.Thread(target=self._activate, daemon=True).start()

    # -- conversions ---------------------------------------------------
    def raw_to_width(self, raw):
        f = (raw - self.raw_open) / float(self.raw_closed - self.raw_open)
        return self.max_open * (1.0 - max(0.0, min(1.0, f)))

    def width_to_raw(self, w):
        f = 1.0 - max(0.0, min(self.max_open, w)) / self.max_open
        return int(round(self.raw_open + f * (self.raw_closed - self.raw_open)))

    # -- polling / publishing -------------------------------------------
    def _poll(self):
        try:
            s = self.drv.status()
        except Exception as e:  # noqa: BLE001
            if self._comm_ok:
                self.get_logger().error(f'communication lost: {e}')
            self._comm_ok = False
            self._publish_diag(None)
            return
        if not self._comm_ok:
            self.get_logger().info('communication restored')
        self._comm_ok = True
        self.status = s
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint]
        js.position = [self.raw_to_width(s.pos)]
        js.effort = [float(s.current)]
        self.pub_js.publish(js)
        self._publish_diag(s)

    def _publish_diag(self, s):
        m = DiagnosticStatus(name='robotiq_2f_85', hardware_id=self.port)
        if s is None:
            m.level, m.message = DiagnosticStatus.ERROR, 'no communication'
        else:
            lvl = DiagnosticStatus.OK
            if not s.activated:
                lvl = DiagnosticStatus.WARN
            if s.fault not in (0, 15):
                lvl = DiagnosticStatus.ERROR
            m.level = lvl
            m.message = FAULTS.get(s.fault, f'fault {s.fault}')
            m.values = [KeyValue(key=k, value=str(v)) for k, v in (
                ('activated', s.activated), ('object', OBJ[s.obj]), ('fault', s.fault),
                ('pos_raw', s.pos), ('pos_req_raw', s.pos_req), ('current_raw', s.current),
                ('width_m', f'{self.raw_to_width(s.pos):.4f}'))]
        self.pub_diag.publish(m)

    # -- activation / services ------------------------------------------
    def _activate(self, timeout=15.0):
        self._activating.set()
        try:
            self.drv.reset()
            time.sleep(0.3)
            self.drv.request_activate()
            t0 = time.time()
            while time.time() - t0 < timeout:
                time.sleep(0.2)
                if self.status and self.status.activated and self.status.fault == 0:
                    return True, 'activated'
            return False, f'activation timeout, status={self.status}'
        except Exception as e:  # noqa: BLE001
            return False, str(e)
        finally:
            self._activating.clear()

    def _srv_activate(self, req, res):
        res.success, res.message = self._activate()
        return res

    def _srv_reset(self, req, res):
        try:
            self.drv.reset()
            res.success, res.message = True, 'reset sent (gripper now needs activation)'
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, str(e)
        return res

    def _srv_stop(self, req, res):
        try:
            self.drv.stop()
            res.success, res.message = True, 'stopped'
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, str(e)
        return res

    # -- GripperCommand action --------------------------------------------
    def _execute(self, goal_handle):
        cmd = goal_handle.request.command
        result = GripperCommand.Result()
        s = self.status
        if s is None or not s.activated or s.fault != 0:
            goal_handle.abort()
            self.get_logger().warn('goal rejected: gripper not activated / faulted')
            return self._fill(result, False)
        raw = self.width_to_raw(cmd.position)
        force = self.def_force
        if cmd.max_effort > 0.0:
            force = int(255 * min(cmd.max_effort, self.max_effort_n) / self.max_effort_n)
        force = min(force, self.max_force)
        speed = min(self.def_speed, self.max_speed)
        try:
            self.drv.move(raw, speed, force)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(str(e))
            goal_handle.abort()
            return self._fill(result, False)
        t0 = time.time()
        while time.time() - t0 < 8.0:
            if goal_handle.is_cancel_requested:
                self.drv.stop()
                goal_handle.canceled()
                return self._fill(result, False)
            time.sleep(0.05)
            s = self.status
            if s is None:
                continue
            fb = GripperCommand.Feedback()
            fb.position, fb.effort = self.raw_to_width(s.pos), float(s.current)
            fb.stalled, fb.reached_goal = s.object_detected, s.obj == 3
            goal_handle.publish_feedback(fb)
            if s.obj != 0 and s.pos_req == raw:
                goal_handle.succeed()
                return self._fill(result, s.obj == 3)
        self.drv.stop()
        goal_handle.abort()
        return self._fill(result, False)

    def _fill(self, result, reached):
        s = self.status
        if s:
            result.position = self.raw_to_width(s.pos)
            result.effort = float(s.current)
            result.stalled = s.object_detected
        result.reached_goal = reached
        return result

    def shutdown(self):
        try:
            self.drv.close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
