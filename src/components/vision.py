import math
import numpy as np

from photonlibpy.photonCamera import PhotonCamera
from wpimath.filter import MedianFilter
from magicbot import feedback


class Vision:
    camera: PhotonCamera
    # size of filter window: larger values are less accurate but better at filtering
    filter_window: int
    # vector from center of robot to camera
    rcx: float
    rcy: float

    _x = 0
    _y = 0
    _z = 0
    _id = 0
    _latency = 0
    sought_ids = []

    def setup(self):
        # setup() required because variables need to be injected
        self._x_filter = MedianFilter(self.filter_window)
        self._y_filter = MedianFilter(self.filter_window)
        self._z_filter = MedianFilter(self.filter_window)

        """If no targets are found within a certain window of time, 
        this component will consider there to be no targets. 
        This is to prevent momentary lapses in detection from 
        causing the robot to jerk
        """
        self.drought = self.filter_window
        # ignores camera height
        self.rc = np.array([self.rcx, self.rcy, 0])
        # CHANGE THIS!!!
        self.sought_ids = [8]

    def hasTargets(self) -> bool:
        return self.drought < self.filter_window

    def getX(self) -> float | None:
        if self.drought < self.filter_window:
            return self._x
        return None

    def getY(self) -> float | None:
        if self.drought < self.filter_window:
            return self._y
        return None

    def getZ(self) -> float | None:
        if self.drought < self.filter_window:
            return self._z
        return None

    def getId(self) -> int | None:
        if self.drought < self.filter_window:
            return self._id
        return None

    def getLatency(self) -> float | None:
        if self.drought < self.filter_window:
            return self._latency
        return None

    # returns angle that robot must turn to face tag
    def getHeading(self) -> float | None:
        if self.drought < self.filter_window:
            return math.atan2(-self._y, self._x) * 180 / math.pi
        return None

    def getAdjustedHeading(self) -> float | None:
        """Returns the angle from the center of the robot to the tag"""
        if self.drought < self.filter_window:
            ct = np.array([self._x, self._y, self._z])
            rt = self.rc + ct
            theta = math.atan2(rt[1], rt[0])
            theta *= 180 / math.pi
            return -theta
        return None

    def setSoughtIds(self, sought_ids):
        self.sought_ids = sought_ids

    def execute(self):
        result = self.camera.getLatestResult()
        if result.hasTargets():
            self.drought = 0
            potential_targets = filter(
                lambda t: t.getFiducialId() in self.sought_ids, result.getTargets()
            )
            target = min(potential_targets, key=lambda t: t.getPoseAmbiguity())
            transform = target.getBestCameraToTarget()
            self._id = target.getFiducialId()
            self._latency = result.getLatencyMillis() / 1000
            self._x = self._x_filter.calculate(transform.X())
            self._y = self._y_filter.calculate(transform.Y())
            self._z = self._z_filter.calculate(transform.Z())
        else:
            self.drought += 1

    @feedback
    def get_id(self) -> int:
        id = self.getId()
        if id is not None:
            return id
        return -1

    @feedback
    def get_x(self) -> int:
        if self.hasTargets():
            return self.getX()
        return 0

    @feedback
    def get_y(self) -> int:
        if self.hasTargets():
            return self.getY()
        return 0

    @feedback
    def get_z(self) -> int:
        if self.hasTargets():
            return self.getZ()
        return 0

    @feedback
    def get_heading(self) -> int:
        if self.hasTargets():
            return self.getHeading()
        return 0

    @feedback
    def get_adjusted_heading(self) -> int:
        if self.hasTargets():
            return self.getAdjustedHeading(
                np.array([self.getX(), self.getY(), self.getZ()])
            )
        return 0
