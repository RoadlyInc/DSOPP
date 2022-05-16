import numpy as np
import enum


class ModelType(enum.Enum):
    pinhole = 0
    radial = 1


class Pinhole:
    """
    Pinhole camera model
    """

    def __init__(self, focal_x, focal_y, center_x, center_y):
        """
        Intialize Pinhole camera via 3x3 calibration matrix
        """

        self.K = np.eye(3)
        self.K[([0, 1, 0, 1], [0, 1, 2,
                               2])] = focal_x, focal_y, center_x, center_y

        self.Kinv = np.linalg.inv(self.K)

    def __str__(self):
        return f'{self.K[0, 0]} {self.K[1, 1]} {self.K[0, 2]} {self.K[1, 2]}'

    def json(self):
        return {
            "fx": self.K[0, 0],
            "fy": self.K[1, 1],
            "cx": self.K[0, 2],
            "cy": self.K[1, 2]
        }

    def project(self, point3d):
        """
        Projects 3d point onto image plane
        Returns 2d point or None if projection is not defined

        Arguments:
            point3d -- 3d point
        """
        image_point = self.K @ point3d
        if image_point[2] <= 0:
            return None
        return image_point[:2] / image_point[2]

    def unproject(self, point2d):
        """
        Unprojects 2d point from image to ray
        Returns ray

        Arguments:
            point2d -- 2d point on the image
        """
        return self.Kinv @ np.append(point2d, 1)


class CameraCalibration:
    """
    Camera calibration
    """

    def __init__(self, intrinsics, image_size, model_type):
        if model_type.name == 'pinhole':
            self.model = Pinhole(*intrinsics[:4])
        else:
            assert False, 'Unsupported camera type'
        self.model_type = model_type.name
        self.image_size = image_size

    def __str__(self):
        return f'{int(self.image_size[0])} {int(self.image_size[1])} {self.model.__str__()}'

    def json(self):
        json_params = self.model.json()
        json_params["width"] = self.image_size[0]
        json_params["height"] = self.image_size[1]
        return json_params

    def project(self, point3d):
        """
        Projects 3d point onto image plane
        Returns 2d point or None if projection is not defined

        Arguments:
            point3d -- 3d point
        """
        point2d = self.model.project(point3d)
        if point2d is None: return None
        if point2d[0] < 0 or point2d[1] < 0: return None
        if point2d[0] > self.image_size[0] - 1: return None
        if point2d[1] > self.image_size[1] - 1: return None

        return point2d
