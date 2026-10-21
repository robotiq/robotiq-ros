"""Gripper backend on the ROS 2 driver, through rclpy.

One node per gripper, created inside the gripper's namespace so that
`robotiq_gripper_controller/gripper_cmd`, `joint_states` and `robot_description`
resolve to that gripper's controller. Each node spins on its own single-threaded
executor in a background thread; the MCP tools run in their own threads and
wait on futures. Single-threaded because rclpy's MultiThreadedExecutor cannot
keep up with the driver's 500 Hz joint_states under the GIL and starves the tool
threads of it. One per node because an executor serves its nodes in no fixed
order, and the pads' frames sharing one would, on some starts, keep the
gripper's own topics from ever being read.

The command joint's name and range are read from `robot_description`, the URDF
robot_state_publisher latches for the cell, so a prefixed cell resolves by itself
and nothing is copied from a datasheet. It is resolved on first use and kept.

The action type is whatever the running controller advertises for `gripper_cmd`:
`ParallelGripperCommand` from Jazzy's parallel_gripper_action_controller,
`GripperCommand` from Humble's position_controllers one. It is read off the graph
rather than guessed from the distro or from what imports, because recent
control_msgs releases ship both types on every distro.

Calls on one gripper are serialised: FastMCP runs tools on worker threads and an
agent may issue two moves at once, and two goals in flight on one controller
resolve in whatever order it likes. The action calls themselves run on the
node's executor thread, never on the tool's: older Humble rclpy registers a goal
request's future after sending it, so a response the executor handled in between
was dropped and the move reported no goal response although the gripper moved.
One deadline covers the call from goal response to result, started once the
action server is known ready, so the tool-level ceiling is the datasheet's
timeout and the first move after start does not spend it on discovery. A goal
that outlives it is cancelled and the cancel's answer is reported, because a
rejected cancel means the gripper is still moving after the tool said it failed.

The effort is 0.0 to 1.0, the gripper's minimum to maximum force, the way the
gripper SDK commands it: the force register has no newton equivalent, since the
force applied depends on the speed, the fingers and the object. The controller's
goal still carries `max_effort` in nominal newtons and the driver divides it by
its `gripper_max_force` to fill the register, so the effort is sent as that
fraction of the `gripper_max_force` the robot description gives the driver. The
speed likewise: a fraction of the gripper's full speed, sent on the goal as
`command.velocity`, that fraction of `gripper_max_speed`. Temporary:
robotiq/robotiq-ros#70 has the driver take the SDK's effort directly, and this scaling
goes with it. A move that names no effort sends the model's default; one that
names no speed runs at the controller's own.

Humble's controller applies neither: `GripperCommand` has no speed field, and
the controller cannot claim the driver's effort interface, so the force comes
from the driver's `gripper_force_multiplier`. There a move that names either
still runs, and its detail says what was not applied. A controller without a
speed interface (the topic-based sim configs) drops the speed without saying
so, which the server cannot see.

A position is reported only from fresh `joint_states` or the controller's
result; a move that failed with neither reports none, never the commanded one.

`holding_effort` is never filled. `joint_states.effort` would be a joint torque
in N m, and no shipped description exports an effort interface anyway; the
gripper reports motor current (gCU), and turning that into a fingertip force
needs a calibration nobody has done.
"""

import threading
import time
from collections.abc import Callable

import rclpy
from control_msgs.action import GripperCommand, ParallelGripperCommand
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from gripper_mcp.backend import BackendHealth, BackendMotion, BackendState
from gripper_mcp.robot_description import DriverScale, command_joint, driver_scale
from gripper_mcp.units import JointGeometry
from gripper_mcp.ros_messages import (
    JOINT_STATES_TOPIC,
    advertised_type,
    cancel_note,
    effort_ignored_message,
    health_from,
    motion_from_result,
    no_state_message,
    position_of,
    refused,
    result_position_of,
    speed_ignored_message,
    timed_out,
    with_note,
)

ACTION_NAME = "robotiq_gripper_controller/gripper_cmd"
ACTION_TYPES = {
    "control_msgs/action/ParallelGripperCommand": ParallelGripperCommand,
    "control_msgs/action/GripperCommand": GripperCommand,
}
DESCRIPTION_TOPIC = "robot_description"
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SERVER_WAIT_S = 2.0
DESCRIPTION_WAIT_S = 2.0
STATE_WAIT_S = 2.0
STALE_STATE_S = 2.0
POLL_S = 0.05


class RosGraph:
    _lock = threading.Lock()
    _executors: list[SingleThreadedExecutor] = []
    _initialised = False

    @classmethod
    def node(cls, name: str, namespace: str) -> Node:
        with cls._lock:
            if not rclpy.ok():
                rclpy.init()
                cls._initialised = True
        return Node(name, namespace=namespace or "/")

    @classmethod
    def spin(cls, node: Node) -> SingleThreadedExecutor:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        with cls._lock:
            cls._executors.append(executor)
        threading.Thread(
            target=executor.spin, name=f"rclpy-spin-{node.get_name()}", daemon=True
        ).start()
        return executor

    @classmethod
    def shutdown(cls) -> None:
        with cls._lock:
            for executor in cls._executors:
                executor.shutdown()
            if cls._initialised:
                rclpy.shutdown()
            cls._executors = []
            cls._initialised = False


def wait_for(future, timeout_s: float):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout_s):
        return None
    return future.result()


def poll_until(probe: Callable[[], object], timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while True:
        found = probe()
        if found is not None or time.monotonic() >= deadline:
            return found
        time.sleep(POLL_S)


def remaining(deadline: float) -> float:
    return max(0.0, deadline - time.monotonic())


def cancel_when_accepted(goal_future) -> None:
    def cancel(done) -> None:
        handle = done.result()
        if handle.accepted:
            handle.cancel_goal_async()

    goal_future.add_done_callback(cancel)


def call_on(executor: SingleThreadedExecutor, call: Callable[[], object], timeout_s):
    return wait_for(executor.create_task(call), timeout_s)


def then(future, timeout_s: float):
    return None if future is None else wait_for(future, timeout_s)


class JointStateFeed:
    def __init__(self, node: Node) -> None:
        self.latest: JointState | None = None
        self.latest_at = 0.0
        self._joint_name: str | None = None
        node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10
        )

    def follow(self, joint_name: str) -> None:
        self._joint_name = joint_name

    def age_s(self) -> float | None:
        if self.latest is None:
            return None
        return time.monotonic() - self.latest_at

    def fresh(self) -> bool:
        age = self.age_s()
        return age is not None and age < STALE_STATE_S

    def _on_joint_states(self, message: JointState) -> None:
        if self._joint_name is not None and self._joint_name in message.name:
            self.latest = message
            self.latest_at = time.monotonic()


class RosGripperBackend:
    name = "ros"

    def __init__(
        self, gripper_name: str, namespace: str, default_effort: float
    ) -> None:
        self._node = RosGraph.node(f"gripper_mcp_{gripper_name}", namespace)
        self._description: str | None = None
        self._node.create_subscription(
            String, DESCRIPTION_TOPIC, self._on_description, LATCHED
        )
        self._states = JointStateFeed(self._node)
        self._joint: JointGeometry | None = None
        self._scale: DriverScale | None = None
        self._default_effort = default_effort
        self._client: ActionClient | None = None
        self._action_type = None
        self._client_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._executor = RosGraph.spin(self._node)

    def joint_geometry(self) -> JointGeometry:
        if self._joint is None:
            if not self._await_description():
                raise RuntimeError(
                    f"No {DESCRIPTION_TOPIC} under '{self._node.get_namespace()}' "
                    f"within {DESCRIPTION_WAIT_S:.0f} s; is robot_state_publisher "
                    "running there?"
                )
            self._joint = command_joint(self._description)
            self._scale = driver_scale(self._description)
            self._states.follow(self._joint.name)
        return self._joint

    def read_state(self) -> BackendState:
        joint = self.joint_geometry().name
        if not self._await_state():
            age = self._states.age_s()
            if age is None or age >= STALE_STATE_S:
                raise RuntimeError(
                    no_state_message(
                        age,
                        joint,
                        self._node.get_namespace(),
                        STATE_WAIT_S,
                        STALE_STATE_S,
                    )
                )
        return BackendState(position_rad=position_of(self._states.latest, joint, 0.0))

    def move_to(
        self,
        position_rad: float,
        effort: float | None,
        timeout_s: float,
        speed: float | None = None,
    ) -> BackendMotion:
        with self._motion_lock:
            return self._move_to(position_rad, effort, timeout_s, speed)

    def _move_to(
        self,
        position_rad: float,
        effort: float | None,
        timeout_s: float,
        speed: float | None,
    ) -> BackendMotion:
        self.joint_geometry()
        client = self._ready_client()
        if client is None:
            return refused(
                self._fresh_position(),
                f"No {ACTION_NAME} action server under "
                f"'{self._node.get_namespace()}'; is the controller active?",
            )
        deadline = time.monotonic() + timeout_s

        note = self._not_applied(effort, speed)
        if self._action_type is not ParallelGripperCommand:
            speed = None
        sent_effort = self._default_effort if effort is None else effort
        goal = self._goal(position_rad, sent_effort, speed)
        motion = self._send(client, goal, deadline, timeout_s)
        return with_note(motion, note)

    def _not_applied(self, effort: float | None, speed: float | None) -> str:
        if self._action_type is ParallelGripperCommand:
            return ""
        action = self._action_type.__name__
        note = effort_ignored_message(action) if effort is not None else ""
        if speed is not None:
            note += speed_ignored_message(action)
        return note

    def _send(
        self,
        client: ActionClient,
        goal,
        deadline: float,
        timeout_s: float,
    ) -> BackendMotion:
        sent = call_on(
            self._executor, lambda: client.send_goal_async(goal), remaining(deadline)
        )
        handle = then(sent, remaining(deadline))
        if handle is None:
            if sent is not None:
                cancel_when_accepted(sent)
            return timed_out(self._fresh_position(), timeout_s, "goal response")
        if not handle.accepted:
            return refused(self._fresh_position(), "The controller rejected the goal.")

        asked = call_on(self._executor, handle.get_result_async, remaining(deadline))
        wrapped = then(asked, remaining(deadline))
        if wrapped is None:
            cancelling = call_on(self._executor, handle.cancel_goal_async, STATE_WAIT_S)
            return timed_out(
                self._fresh_position(),
                timeout_s,
                "result",
                cancel_note(then(cancelling, STATE_WAIT_S)),
            )

        return motion_from_result(
            wrapped.status,
            self._result_position(wrapped.result, self._fresh_position()),
            wrapped.result.stalled,
            wrapped.result.reached_goal,
        )

    def health(self) -> BackendHealth:
        client = self._ready_client()
        problem = self._description_problem()
        state_fresh = problem is None and self._await_state()
        return health_from(
            action_ready=client is not None,
            state_fresh=state_fresh,
            detail=(
                f"{ACTION_NAME} {self._describe(client)}, "
                f"{JOINT_STATES_TOPIC} {'fresh' if state_fresh else 'stale'}, "
                f"{DESCRIPTION_TOPIC} "
                f"{'received' if self._description else 'absent'} "
                f"under '{self._node.get_namespace()}'."
                + ("" if problem is None else f" {problem}")
            ),
        )

    def _description_problem(self) -> str | None:
        try:
            self.joint_geometry()
        except (RuntimeError, ValueError) as error:
            return str(error)
        return None

    def _on_description(self, message: String) -> None:
        self._description = message.data

    def _fresh_position(self) -> float | None:
        latest = self._states.latest
        if latest is None or not self._states.fresh():
            return None
        return position_of(latest, self._joint.name, None)

    def _await_description(self) -> bool:
        return poll_until(lambda: self._description, DESCRIPTION_WAIT_S) is not None

    def _await_state(self) -> bool:
        return (
            poll_until(lambda: self._states.fresh() or None, STATE_WAIT_S) is not None
        )

    def _ready_client(self) -> ActionClient | None:
        with self._client_lock:
            if self._client is None:
                type_name = poll_until(self._advertised_type, SERVER_WAIT_S)
                if type_name is None:
                    return None
                self._action_type = ACTION_TYPES[type_name]
                self._client = ActionClient(self._node, self._action_type, ACTION_NAME)
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            return None
        return self._client

    def _advertised_type(self) -> str | None:
        return advertised_type(
            get_action_names_and_types(self._node),
            self._resolved_action_name(),
            ACTION_TYPES,
        )

    def _resolved_action_name(self) -> str:
        return self._node.get_namespace().rstrip("/") + "/" + ACTION_NAME

    def _describe(self, client: ActionClient | None) -> str:
        if client is None:
            return "absent"
        return f"ready ({self._action_type.__name__})"

    def _goal(self, position_rad: float, effort: float, speed: float | None):
        goal = self._action_type.Goal()
        nominal_n = effort * self._scale.max_force_n
        if self._action_type is ParallelGripperCommand:
            goal.command.name = [self._joint.name]
            goal.command.position = [position_rad]
            goal.command.effort = [nominal_n]
            if speed is not None:
                goal.command.velocity = [speed * self._scale.max_speed_m_s]
        else:
            goal.command.position = position_rad
            goal.command.max_effort = nominal_n
        return goal

    def _result_position(self, result, fallback: float | None) -> float | None:
        if self._action_type is ParallelGripperCommand:
            return result_position_of(result.state, self._joint.name, fallback)
        return result.position
