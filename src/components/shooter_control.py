from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state, timed_state

from components.intake import Intake
from components.shooter import Shooter


class ShooterControl(StateMachine):
    """State machine that controls the shooter and shooter feed on the
    robot
    """

    # other components
    intake: Intake
    shooter: Shooter

    intake_trigger = will_reset_to(False)
    eject_trigger = will_reset_to(False)
    shoot_trigger = will_reset_to(False)
    intake_state = "transitioning"

    def update_intake_state(self, state: str):
        self.intake_state = state

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
        if self.eject_trigger:
            self.next_state("ejecting")
            return
        if self.intake_trigger:
            self.next_state("intaking")
            return
        if self.shoot_trigger:
            self.next_state("loading")

    @state
    def intaking(self):
        self.shooter.intake()
        self.shooter.feed_in()
        if self.eject_trigger:
            self.next_state("ejecting")
            return
        if not self.intake_trigger:
            self.next_state("idle")

    @state
    def ejecting(self):
        self.shooter.eject()
        if not self.eject_trigger:
            self.next_state("idle")

    @timed_state(duration=0.25, next_state="preshooting")
    def loading(self):
        self.shooter.feed_out()

    @timed_state(duration=0.75, next_state="shooting")
    def preshooting(self):
        self.shooter.shoot()

    @timed_state(duration=1.0, next_state="idle")
    def shooting(self):
        self.shooter.feed_in()
        self.shooter.shoot()
