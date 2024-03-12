import math
import numpy as np

from photonlibpy.photonCamera import PhotonCamera
from wpimath.filter import MedianFilter
from magicbot import feedback

from util import compensate


class SmartCamera(PhotonCamera):
    """Wrapper for photonlibpy PhotonCamera that adds the following features:
    1. Filters on x, y, and z values
    2. Protection against brief lapses in tag detection
    3. Heading of tag relative to camera and relative to robot
    4. Retrieval of tags by ID"""

    def __init__(
        self,
        camera_name: str,
        rc: np.array,
        tilt: float = 0,
        filter_window: int = 10,
        sought_ids: list[int] = [],
    ):
        """Parameters:
        camera_name -- name of camera in PhotonVision
        rc -- vector from center of robot to camera. z-coord ignored
        tilt -- pitch of camera in degrees. Tilt up is positive
        filter_window -- size of window on MedianFilters. Also the
            number of ticks until a tag is considered lost. Higher
            values yielf less noisy and spotty data but with less
            accuracy and precision and higher latency.
        sought_ids -- tag ids the camera will look for. Tags that it
            finds with non-sought ids will be ignored.
        """
        PhotonCamera.__init__(self, camera_name)
        self.rc = rc
        self.tilt = tilt
        self.filter_window = filter_window
        self.x = 0
        self.y = 0
        self.z = 0
        self.x_filter = MedianFilter(self.filter_window)
        self.y_filter = MedianFilter(self.filter_window)
        self.z_filter = MedianFilter(self.filter_window)
        self.latency = 0
        self.drought = self.filter_window
        self.sought_ids = sought_ids

    def update(self) -> None:
        """Call this every loop"""
        result = self.getLatestResult()
        if result.hasTargets():
            self.drought = 0
            potential_targets = list(
                filter(
                    lambda t: t.getFiducialId() in self.sought_ids, result.getTargets()
                )
            )
            if len(potential_targets) == 0:
                self.drought += 1
                return
            target = min(potential_targets, key=lambda t: t.getPoseAmbiguity())
            transform = target.getBestCameraToTarget()
            self.id = target.getFiducialId()
            self.latency = result.getLatencyMillis() / 1000
            u = transform.X()
            v = transform.Y()
            w = transform.Z()
            theta = self.tilt * math.pi / 180
            self.x = self.x_filter.calculate(u * math.cos(theta) - w * math.sin(theta))
            self.y = self.y_filter.calculate(v)
            self.z = self.z_filter.calculate(u * math.sin(theta) + w * math.cos(theta))
        else:
            self.drought += 1

    def hasTargets(self) -> bool:
        return self.drought < self.filter_window

    def getX(self) -> float | None:
        if self.drought < self.filter_window:
            return self.x
        return None

    def getY(self) -> float | None:
        if self.drought < self.filter_window:
            return self.y
        return None

    def getZ(self) -> float | None:
        if self.drought < self.filter_window:
            return self.z
        return None

    def getId(self) -> int | None:
        if self.drought < self.filter_window:
            return self.id
        return None

    def getLatency(self) -> float | None:
        if self.drought < self.filter_window:
            return self.latency
        return None

    # returns angle that robot must turn to face tag
    def getHeading(self) -> float | None:
        if self.drought < self.filter_window:
            return math.atan2(-self.y, self.x) * 180 / math.pi
        return None

    def getAdjustedHeading(self) -> float | None:
        """Returns the angle from the center of the robot to the tag"""
        if self.drought < self.filter_window:
            ct = np.array([self.x, self.y, self.z])
            rt = self.rc + ct
            theta = math.atan2(rt[1], rt[0])
            theta *= 180 / math.pi
            return -theta
        return None

    def setSoughtIds(self, sought_ids):
        self.sought_ids = sought_ids


class Vision:
    left_camera: SmartCamera
    right_camera: SmartCamera

    sought_ids = []

    def setup(self):
        self.cameras = [self.left_camera, self.right_camera]

    def hasTargets(self) -> bool:
        return self.left_camera.hasTargets() or self.right_camera.hasTargets()

    def getX(self) -> float | None:
        return compensate([cam.getX() for cam in self.cameras])

    def getY(self) -> float | None:
        return compensate([cam.getY() for cam in self.cameras])

    def getZ(self) -> float | None:
        return compensate([cam.getZ() for cam in self.cameras])

    def getLatency(self) -> float | None:
        return compensate([cam.getLatency() for cam in self.cameras])

    # returns angle that robot must turn to face tag
    def getHeading(self) -> float | None:
        return compensate([cam.getHeading() for cam in self.cameras])

    def getAdjustedHeading(self) -> float | None:
        return compensate([cam.getAdjustedHeading() for cam in self.cameras])

    def setSoughtIds(self, sought_ids):
        self.sought_ids = sought_ids

    def execute(self):
        for cam in self.cameras:
            cam.setSoughtIds(self.sought_ids)
            cam.update()

    # @feedback
    # def get_left_id(self) -> int:
    #     id = self.left_camera.getId()
    #     if id is not None:
    #         return id
    #     return -1

    # @feedback
    # def get_right_id(self) -> int:
    #     id = self.right_camera.getId()
    #     if id is not None:
    #         return id
    #     return -1

    @feedback
    def get_x(self) -> float:
        x = self.getX()
        if self.hasTargets() and x is not None:
            return x
        return 0

    @feedback
    def get_left_x(self) -> float:
        x = self.left_camera.getX()
        if self.hasTargets():
            return x
        return 0

    @feedback
    def get_right_x(self) -> float:
        x = self.right_camera.getX()
        if self.hasTargets():
            return x
        return 0

    @feedback
    def get_y(self) -> float:
        y = self.getY()
        if self.hasTargets() and y is not None:
            return y
        return 0

    @feedback
    def get_z(self) -> float:
        z = self.getZ()
        if self.hasTargets() and z is not None:
            return z
        return 0

    @feedback
    def get_heading(self) -> float:
        heading = self.getHeading()
        if self.hasTargets() and heading is not None:
            return heading
        return 0

    @feedback
    def get_adjusted_heading(self) -> float:
        heading = self.getAdjustedHeading()
        if self.hasTargets() and heading is not None:
            return heading
        return 0
