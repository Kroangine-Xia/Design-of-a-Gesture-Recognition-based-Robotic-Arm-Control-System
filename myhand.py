#!/usr/bin/env python

import numpy as np
from spatialmath.base import trotz, transl
from roboticstoolbox import DHRobot, RevoluteDH, RevoluteMDH


class Myhand(DHRobot):
    """
    A class representing the Panda robot arm.

    ``Panda()`` is a class which models a Franka-Emika Panda robot and
    describes its kinematic characteristics using modified DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.Panda()
        >>> print(robot)

    .. note::
        - SI units of metres are used.
        - The model includes a tool offset.

    :references:
        - https://frankaemika.github.io/docs/control_parameters.html

    .. codeauthor:: Samuel Drew
    .. codeauthor:: Peter Corke
    """

    def __init__(self):

        # deg = np.pi/180
        mm = 1e-3
        tool_offset = (103) * mm

        flange = (107) * mm
        # d7 = (58.4)*mm

        # This Panda model is defined using modified
        # Denavit-Hartenberg parameters
        L = [
            RevoluteDH(
                a=0.0,
                d=0.0,
                alpha=np.pi / 2,
                #offset=10,
                qlim=np.array([-2.8973, 2.8973])
            ),
            RevoluteDH(
                a=0.2,
                d=0.0,
                alpha=-np.pi / 2,
                qlim=np.array([-1.7628, 1.7628])
            ),
            RevoluteDH(
                a=0.0,
                d=0.0,
                alpha=np.pi / 2,
                qlim=np.array([-2.8973, 2.8973])
            ),
            RevoluteDH(
                a=0.0,
                d=0.2,
                alpha=0,
                qlim=np.array([-3.0718, -0.0698])
            )
        ]

        tool = transl(0, 0, tool_offset) @ trotz(-np.pi / 4)

        super().__init__(
            L,
            name="hand"
        )

        self.qr = np.array([np.pi / 4, np.pi / 6, np.pi / 4, 0]) #开始的位姿 前三个是坐标 后四个是四元数
        self.qr2 = np.array([0, np.pi / 4, 0, 0])  # 开始的位姿 前三个是坐标 后四个是四元数
        self.qz = np.zeros(4)
        self.qz = np.array([0, 0, np.pi / 2, 0])  # 开始的位姿 前三个是坐标 后四个是四元数

        #self.qr = np.zeros(4)

        #self.qz = np.array([1, -1.3, 3, -2.2, 0, 2.0, np.pi / 2])#结束的位姿

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    panda = Myhand()
    print(panda)
