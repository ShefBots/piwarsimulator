from time import monotonic as time
from controllers.SimulatedGripperController import SimulatedGripperController
from world.ObjectType import *

from devices import IOController


# base off SimulatedGripperController but just don't use any of the simulation bits of it
class GripperController(SimulatedGripperController):
    """move the real gripper"""

    def __init__(self, robot, serial_instances):
        print("Initialising GripperController...")
        super(GripperController, self).__init__(robot)

        assert robot.object_type == ObjectType.ROBOT
        self.robot = robot

        self.io_controller = None

        if IOController.EXPECTED_ID in serial_instances.keys():
            self.io_controller = IOController(
                serial_instances[IOController.EXPECTED_ID]
            )
        else:
            raise Exception("Could not find IO controller hardware")

        # assume starting with the gripper closed
        self.gripper_angle = 0
        self._gripper_state = self.GRIPPER_CLOSED
        self.last_gripper_state = time()

    @property
    def gripper_state(self):
        # property has a timeout and only checks for a new value every 0.5s or os
        # and otherwise returns cached self._gripper_state
        # also updates self.gripper_angle (45/2 if UNKNOWN)
        if time() - self.last_gripper_state > 0.5:
            # refresh cache
            self._gripper_state = self.io_controller.gripper_state()
            self.last_gripper_state = time()
            # update angle
            if self._gripper_state == self.GRIPPER_CLOSED:
                self.gripper_angle = 0
            elif self._gripper_state == self.GRIPPER_OPEN:
                self.gripper_angle = self.GRIPPER_ANGLE_OPEN
            if self._gripper_state == self.GRIPPER_UNKOWN:
                self.gripper_angle = self.GRIPPER_ANGLE_OPEN / 2

        # return the cached value
        return self._gripper_state

    def open_gripper(self):
        """open the gripper"""
        if (
            self.gripper_state == self.GRIPPER_OPEN
            or self.gripper_state == self.GRIPPER_OPENING
        ):
            return
        print("Opening gripper")
        self.io_controller.open_gripper()

    def close_gripper(self):
        """close the gripper"""
        if (
            self.gripper_state == self.GRIPPER_CLOSED
            or self.gripper_state == self.GRIPPER_CLOSING
        ):
            return
        print("Closing gripper")
        self.io_controller.close_gripper()
