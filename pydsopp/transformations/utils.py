import numpy as np
import sympy

from sympy.algebras.quaternion import Quaternion
from .motion import Motion


def sim3_from_parameters(parameters):
    """
    create Sim3 matrix from the format: quaternion, translation
    Arguments:
        parameters parameters in order: quaternion, translation
    Returns matrix of transformation
    """
    quaternion = Quaternion(parameters[3], *parameters[:3])
    translation = np.array(parameters[4:])
    return Motion(quaternion, translation)


def se3_from_parameters(parameters):
    """
    create Se3 motion from the format: quaternion, translation
    NOTE: quaternion would be normalized
    Arguments:
        parameters parameters in order: quaternion, translation
    Returns se3 transformation
    """
    quaternion = Quaternion(parameters[3], *parameters[:3])
    translation = np.array(parameters[4:])
    return Motion(quaternion / quaternion.norm(), translation)
