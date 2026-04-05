# kinematics.py
# Parses robot URDF and provides analytical FK + Jacobian.
# Used by bridge_node for per-frame manipulability computation.

import numpy as np
from urdf_parser_py.urdf import URDF
from typing import List

class URDFKinematics:
    """
    Denavit-Hartenberg forward kinematics from a parsed URDF.
    Supports revolute joints only (sufficient for 6-DOF arm).
    """

    def __init__(self, urdf_path: str, base_link: str, tip_link: str):
        robot = URDF.from_xml_file(urdf_path)
        self._chain = robot.get_chain(base_link, tip_link, joints=True, links=False)
        self._joints = [
            j for j in robot.joints
            if j.name in self._chain and j.type == 'revolute'
        ]
        self._num_joints = len(self._joints)

    def fk(self, q: List[float]) -> np.ndarray:
        """Returns 4×4 homogeneous transform of TCP given joint angles q (rad)."""
        T = np.eye(4)
        for i, joint in enumerate(self._joints):
            # Extract joint axis and origin from URDF
            axis = np.array(joint.axis) if joint.axis else np.array([0, 0, 1])
            origin = joint.origin
            trans = np.array(origin.xyz) if origin else np.zeros(3)
            rpy   = np.array(origin.rpy) if origin else np.zeros(3)

            # Build fixed transform (offset)
            T_offset = self._rpy_to_mat(rpy, trans)
            # Build joint rotation
            T_joint  = self._axis_angle_to_mat(axis, q[i])

            T = T @ T_offset @ T_joint
        return T

    def jacobian(self, q: List[float]) -> np.ndarray:
        """Numerical 6×N Jacobian via central finite differences. Δq = 1e-6 rad."""
        dq = 1e-6
        J = np.zeros((6, self._num_joints))
        T0 = self.fk(q)

        for i in range(self._num_joints):
            q_plus  = list(q); q_plus[i]  += dq
            q_minus = list(q); q_minus[i] -= dq
            T_plus  = self.fk(q_plus)
            T_minus = self.fk(q_minus)

            # Linear velocity columns
            J[0:3, i] = (T_plus[0:3, 3] - T_minus[0:3, 3]) / (2 * dq)
            # Angular velocity columns (from rotation matrix difference → skew-symmetric)
            dR = (T_plus[0:3, 0:3] - T_minus[0:3, 0:3]) / (2 * dq)
            R  = T0[0:3, 0:3]
            S  = dR @ R.T
            J[3, i] = S[2, 1]
            J[4, i] = S[0, 2]
            J[5, i] = S[1, 0]
        return J

    @staticmethod
    def _rpy_to_mat(rpy: np.ndarray, trans: np.ndarray) -> np.ndarray:
        r, p, y = rpy
        Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
        R = Rz @ Ry @ Rx
        T = np.eye(4); T[0:3,0:3] = R; T[0:3,3] = trans
        return T

    @staticmethod
    def _axis_angle_to_mat(axis: np.ndarray, angle: float) -> np.ndarray:
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        c, s = np.cos(angle), np.sin(angle)
        t = 1 - c
        x, y, z = axis
        R = np.array([
            [t*x*x+c,   t*x*y-s*z, t*x*z+s*y],
            [t*x*y+s*z, t*y*y+c,   t*y*z-s*x],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c  ]
        ])
        T = np.eye(4); T[0:3,0:3] = R
        return T
