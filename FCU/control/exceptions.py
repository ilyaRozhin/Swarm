#!/usr/bin/env python3

class ErrorException(Exception):
    """Base class for other exceptions"""
    pass

class TimeoutException(ErrorException):
    pass

class WaitModeTimeout(TimeoutException):
    """Thrown when fails to achieve given mode change."""
    pass

class WaitAltitudeTimout(TimeoutException):
    """Thrown when fails to achieve given altitude range."""
    pass

class WaitGroundSpeedTimeout(TimeoutException):
    """Thrown when fails to achieve given ground speed range."""
    pass

class WaitRollTimeout(TimeoutException):
    """Thrown when fails to achieve given roll in degrees."""
    pass

class WaitPitchTimeout(TimeoutException):
    """Thrown when fails to achieve given pitch in degrees."""
    pass

class WaitHeadingTimeout(TimeoutException):
    """Thrown when fails to achieve given heading."""
    pass

class WaitDistanceTimeout(TimeoutException):
    """Thrown when fails to attain distance"""
    pass

class WaitLocationTimeout(TimeoutException):
    """Thrown when fails to attain location"""
    pass

class WaitWaypointTimeout(TimeoutException):
    """Thrown when fails to attain waypoint ranges"""
    pass

class SetRCTimeout(TimeoutException):
    """Thrown when fails to send RC commands"""
    pass

class MsgRcvTimeoutException(TimeoutException):
    """Thrown when fails to receive an expected message"""
    pass

class NotAchievedException(ErrorException):
    """Thrown when fails to achieve a goal"""
    pass

class YawSpeedNotAchievedException(NotAchievedException):
    """Thrown when fails to achieve given yaw speed."""
    pass

class SpeedVectorNotAchievedException(NotAchievedException):
    """Thrown when fails to achieve given speed vector."""
    pass

class PreconditionFailedException(ErrorException):
    """Thrown when a precondition for a test is not met"""
    pass

class ArmedAtEndOfTestException(ErrorException):
    """Created when test left vehicle armed"""
    pass

