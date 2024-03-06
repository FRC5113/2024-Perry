from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state

from components.intake import Intake
from components.shooter import Shooter


class IntakeControl(StateMachine):
    """State machine that controls the intake on the robot and makes its
    operation safer and easier
    """

    # other components
    intake: Intake
    shooter: Shooter

    """`joint_setpoint` is used instead of directly setting the setpoint
    of the PID controller because it ensures that the setpoint will only
    be changed once the state machine determines it is okay for the
    joint to move. Setting this to -1 will cause the state machine not
    to update the setpoint."""
    joint_setpoint = will_reset_to(-1)
    # eject_trigger will override intake_trigger
    intake_trigger = will_reset_to(False)
    eject_trigger = will_reset_to(False)
    shooter_state = "idle"

    """Control methods
    Note that these are all prefixed with "request." This is because the
    corresponding action is not guaranteed to happen -- it will only
    occur if the statemachine determines it to be correct to do. In
    addition, if a request does not happen for whatever reason, it
    still will not happen when the statemachine determines doing that
    request is okay -- the action must be requested again.
    """

    def request_up(self):
        self.joint_setpoint = self.intake.lower_limit

    def request_down(self):
        self.joint_setpoint = self.intake.upper_limit

    def request_intake(self):
        self.intake_trigger = True

    def request_eject(self):
        self.eject_trigger = True

    def update_shooter_state(self, state: str):
        self.shooter_state = state

    # states
    @state(first=True)
    def transitioning(self):
        if self.shooter_state == "idle" or self.shooter_state == "holding":
            if self.joint_setpoint != -1:
                self.intake.set_joint_setpoint(self.joint_setpoint)
            if not self.intake.is_at_setpoint():
                self.intake.move_to_setpoint()
        if self.intake.is_at_setpoint():
            if self.intake.get_joint_setpoint() == self.intake.lower_limit:
                self.next_state("idle")
            if self.intake.get_joint_setpoint() == self.intake.upper_limit:
                self.next_state("ready")

    @state
    def idle(self):
        if self.shooter_state == "idle" or self.shooter_state == "holding":
            if self.joint_setpoint != -1:
                self.intake.set_joint_setpoint(self.joint_setpoint)
        if (
            not self.intake.is_at_setpoint()
            or not self.intake.get_joint_setpoint() == self.intake.lower_limit
        ):
            self.next_state("transitioning")

    @state
    def ready(self):
        if self.eject_trigger:
            self.next_state("ejecting")
            return
        if self.intake_trigger:
            self.next_state("intaking")
            return
        if self.shooter_state == "idle" or self.shooter_state == "holding":
            if self.joint_setpoint != -1:
                self.intake.set_joint_setpoint(self.joint_setpoint)
        if (
            not self.intake.is_at_setpoint()
            or not self.intake.get_joint_setpoint() == self.intake.upper_limit
        ):
            self.next_state("transitioning")

    @state
    def intaking(self, state_tm):
        # be careful with state_tm - "may not start at zero????"
        if self.eject_trigger:
            self.next_state("ejecting")
            return
        if self.shooter_state == "idle" or self.shooter_state == "intaking":
            self.intake.intake()
        if not self.intake_trigger:
            self.next_state("ready")

    @state
    def ejecting(self, state_tm):
        self.intake.eject()
        if not self.eject_trigger:
            self.next_state("ready")

    @state
    def disabled(self):
        self.intake.disable()
