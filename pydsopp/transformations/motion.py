import sympy
import numpy as np
from sympy.algebras.quaternion import Quaternion
from math import sqrt


class Motion:
    """ 3 dimensional group of similarity transformations """
    def __init__(self, q, t):
        """
        Construct `Motion` from q,t or sympy matrix
        Arguments:
            q -- quaternion
            t -- translation
        """
        assert isinstance(q, Quaternion)
        assert isinstance(t, np.ndarray)

        scale_sqrt = q.norm()
        self.__scale = scale_sqrt**2

        self.__quaternion = q / scale_sqrt
        self.__translation = t

        self.__matrix = np.eye(4)
        self.__matrix[0:3, 0:3] = self.__scale * np.array(
            self.__quaternion.to_rotation_matrix())
        self.__matrix[0:3, 3] = t

    def __mul__(self, right):
        if isinstance(right, Motion):
            q_mul = sqrt(
                self.__scale *
                right.__scale) * self.__quaternion * right.__quaternion
            t_mul = self.__translation + self.__matrix[
                0:3, 0:3] @ right.__translation

            return Motion(q_mul, t_mul)
        elif isinstance(right, np.ndarray):
            if right.shape == (4, ):
                return self.__matrix @ right
            elif right.shape == (3, ):
                return self.__matrix[0:3, 0:3] @ right + self.__matrix[0:3, 3]
            else:
                assert False, f'unsupported shape {right.shape}'
        elif isinstance(right, sympy.Matrix):
            return self.__mul__(np.array(right))
        else:
            assert False, f'unsupported type {type(right)}'

    def __matmul__(self, right):
        return self.__mul__(right)

    def inverse(self):
        q_inv = self.__quaternion.inverse()
        t_inv = -np.array(q_inv.to_rotation_matrix()) @ self.__translation
        return Motion(q_inv / sqrt(self.__scale), t_inv / self.__scale)

    def se3(self):
        """
        Get scale normalized Motion
        """
        return Motion(self.__quaternion, self.__translation)

    def matrix(self):
        return self.__matrix

    def quaternion(self):
        return sqrt(self.__scale) * self.__quaternion

    def translation(self):
        return self.__translation
