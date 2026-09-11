"""Whether the arm is held is a fact about the graph, not about this process.

`arm-pose go` leaves `arm_controller` active on purpose — it leaves the arm
holding the pose it reached — so the *next* command client starts against an arm
that is already holding. `ArmControl` used to assume `holding = False` at
construction, so that second process asked to activate an already-active
controller, the STRICT switch refused, `send` returned False, and the streaming
loop retried at 20 Hz:

    Controller with name 'arm_controller' is already active.
    Aborting, no controller is switched! (::STRICT switch)

Found at the bench on the second `arm-pose go` of a session.
"""

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController

from mote_arm.control import ARM_CONTROLLER, LIST_SERVICE, SWITCH_SERVICE, ArmControl


class FakeFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class FakeClient:
    def __init__(self, answer, available=True):
        self.answer = answer
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec=None):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture(self.answer(request))


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class FakeNode:
    def __init__(self, clients):
        self.clients = clients
        self.logger = FakeLogger()

    def create_publisher(self, *_args, **_kw):
        return self

    def create_client(self, _srv, name):
        return self.clients[name]

    def get_logger(self):
        return self.logger

    def publish(self, _msg):
        pass


def listing(state: str | None):
    def answer(_request):
        response = ListControllers.Response()
        if state is not None:
            controller = ControllerState()
            controller.name = ARM_CONTROLLER
            controller.state = state
            response.controller = [controller]
        return response

    return answer


def switching(ok: bool):
    def answer(_request):
        response = SwitchController.Response()
        response.ok = ok
        return response

    return answer


def control(state="inactive", switch_ok=True, list_available=True):
    clients = {
        SWITCH_SERVICE: FakeClient(switching(switch_ok)),
        LIST_SERVICE: FakeClient(listing(state), available=list_available),
    }
    node = FakeNode(clients)
    return ArmControl(node), clients, node


def test_a_fresh_client_does_not_re_activate_an_already_active_controller():
    """The bug: a second `arm-pose go` in a session, refused every setpoint."""
    arm, clients, _ = control(state="active")
    assert arm.set_holding(True) is True
    assert clients[SWITCH_SERVICE].requests == []


def test_a_fresh_client_activates_a_controller_that_is_inactive():
    arm, clients, _ = control(state="inactive")
    assert arm.set_holding(True) is True
    assert len(clients[SWITCH_SERVICE].requests) == 1
    assert clients[SWITCH_SERVICE].requests[0].activate_controllers == [ARM_CONTROLLER]


def test_a_fresh_client_deactivates_a_controller_another_process_left_holding():
    """A client that says it limps on exit: assuming False meant it silently did not."""
    arm, clients, _ = control(state="active")
    assert arm.set_holding(False) is True
    assert clients[SWITCH_SERVICE].requests[0].deactivate_controllers == [
        ARM_CONTROLLER
    ]


def test_a_refused_switch_is_success_when_the_state_is_already_right():
    """Something else may move the controller between the read and the switch."""
    arm, _, node = control(state="active", switch_ok=False)
    arm.holding = False  # a stale belief, as if seeded before that change
    assert arm.set_holding(True) is True
    assert node.logger.warnings == []


def test_a_refused_switch_is_a_failure_when_the_state_is_still_wrong():
    arm, _, node = control(state="inactive", switch_ok=False)
    assert arm.set_holding(True) is False
    assert node.logger.warnings


def test_an_unreadable_state_still_attempts_the_switch():
    """No answer is not the same as `inactive`; try, rather than assume."""
    arm, clients, _ = control(state="active", list_available=False)
    assert arm.active() is None
    assert arm.set_holding(True) is True
    assert len(clients[SWITCH_SERVICE].requests) == 1


def test_a_controller_missing_from_the_listing_reads_as_unknown():
    arm, _, _ = control(state=None)
    assert arm.active() is None


def test_held_asks_once_and_then_remembers():
    arm, clients, _ = control(state="active")
    assert arm.held is True
    assert arm.held is True
    assert len(clients[LIST_SERVICE].requests) == 1


def test_send_takes_hold_before_it_publishes():
    arm, clients, _ = control(state="inactive")
    assert arm.send({"elbow_flex": 0.1}, 1.0) is True
    assert clients[SWITCH_SERVICE].requests[0].activate_controllers == [ARM_CONTROLLER]


def test_send_reports_failure_rather_than_publishing_into_a_limp_arm():
    arm, _, _ = control(state="inactive", switch_ok=False)
    assert arm.send({"elbow_flex": 0.1}, 1.0) is False
