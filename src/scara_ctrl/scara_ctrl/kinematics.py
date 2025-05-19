import numpy as np
from numpy import sin, cos

class Kinematics:
    def __init__(self, l1=.165, l2=.165):
        self.l1 = l1  # m
        self.l2 = l2  # m

    def forward(self, position: np.array):
        th1, th2, th3 = tuple(position)

        return np.array(
            [
                self.l1 * cos(th1) + self.l2 * cos(th1 + th2),
                self.l1 * sin(th1) + self.l2 * sin(th1 + th2),
                th3,
            ]
        )

    def jacobian(self, position: np.array):
        th1, th2, _ = tuple(position)

        return np.array(
            [
                [
                    -self.l1 * sin(th1) - self.l2 * sin(th1 + th2),
                    -self.l2 * sin(th1 + th2),
                    0,
                ],
                [
                    self.l1 * cos(th1) + self.l2 * cos(th1 + th2),
                    self.l2 * cos(th1 + th2),
                    0,
                ],
                [0, 0, 1],
            ]
        )
