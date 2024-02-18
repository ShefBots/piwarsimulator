#!/usr/bin/env python3


class Controller:
    """controllers are things that move the robot"""

    def stop(self, exiting=False):
        """stop moving"""
        pass

    def __del__(self):
        self.stop(exiting=True)
