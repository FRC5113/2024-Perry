from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state, timed_state

from components.intake import Intake
from components.shooter import Shooter
from components.intake_control import IntakeControl


class ShooterControl(StateMachine):
    """State machine that controls the shooter and shooter feed on the
    robot
    """

    # other components
    intake_control: IntakeControl
    intake: Intake
    shooter: Shooter

    intake_trigger = will_reset_to(False)
    eject_trigger = will_reset_to(False)
    shoot_trigger = will_reset_to(False)

    # informational methods
    def is_running_motors(self):
        return self.current_state != "idle" and self.current_state != "holding"

    def is_intake_ready(self):
        return self.current_state == "idle" or self.current_state == "intaking"

    """Control methods
    Note that these are all prefixed with "request." This is because the
    corresponding action is not guaranteed to happen -- it will only
    occur if the statemachine determines it to be correct to do. In
    addition, if a request does not happen for whatever reason, it 
    still will not happen when the statemachine determines doing that
    request is okay -- the action must be requested again.
    """

    def request_intake(self):
        """This probably should not be called from operator input"""
        self.intake_trigger = True

    def request_eject(self):
        self.eject_trigger = True

    def request_shoot(self):
        self.shoot_trigger = True

    # states
    @state(first=True)
    def idle(self):
        if self.intake_trigger:  # add delay?
            self.next_state("intaking")
        # should theoretically not need a transition to ejecting

    @state
    def intaking(self, state_tm):
        self.shooter.intake()
        # be careful with state_tm
        if self.eject_trigger:
            self.next_state("ejecting")
            return
        if state_tm > 1 and not self.intake.has_note:
            if self.shooter.has_note():
                self.next_state("holding")
            else:
                self.next_state("idle")

    @state
    def ejecting(self, state_tm):
        self.shooter.eject()
        if self.shooter.has_note() and state_tm > 1:
            self.next_state("idle")

    @state
    def holding(self):
        if self.eject_trigger:
            self.next_state("ejecting")
            return
        if self.shoot_trigger:
            self.next_state("feeding")

    @timed_state(duration=1.0, next_state="shooting")
    def feeding(self):
        self.shooter.intake()
        self.shooter.feed()

    @timed_state(duration=1.0, next_state="idle")
    def shooting(self):
        self.shooter.feed()
        self.shooter.shoot()
