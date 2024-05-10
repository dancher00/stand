import sympy as sp
from sympy import BlockDiagMatrix
import numpy as np
import casadi as cs


class Kinematics:
    def __init__(self) -> None:
        # sympy
        angles_sym = sp.MatrixSymbol("a", 3, 1)
        self._transf_matrices_sym = self._forward_kin(angles_sym)
        self._points_sym = [
            self._homogeneous_to_cartesian(mat) for mat in self._transf_matrices_sym
        ]
        self._fk = sp.lambdify(angles_sym, self._points_sym)

        # casadi
        angles_cs = cs.SX.sym("angles", 3)
        self._transf_matrices_cs = self._fk(angles_cs)
        self._fk_cs = cs.Function("fk", [angles_cs], self._transf_matrices_cs)

    def _rotx(self, ang):
        mat = sp.Matrix(
            [
                [1, 0, 0],
                [0, sp.cos(ang), -sp.sin(ang)],
                [0, sp.sin(ang), sp.cos(ang)],
            ]
        )
        return BlockDiagMatrix(mat, sp.Matrix([1]))

    def _roty(self, ang):
        mat = sp.Matrix(
            [
                [sp.cos(ang), 0, sp.sin(ang)],
                [0, 1, 0],
                [-sp.sin(ang), 0, sp.cos(ang)],
            ]
        )
        return BlockDiagMatrix(mat, sp.Matrix([1]))

    def _rotz(self, ang):
        mat = np.array(
            [
                [sp.cos(ang), -sp.sin(ang), 0],
                [sp.sin(ang), sp.cos(ang), 0],
                [0, 0, 1],
            ]
        )
        return BlockDiagMatrix(mat, sp.Matrix([1]))

    def _trans(self, v):
        res = np.eye(4)
        res[:3, -1] = v
        return res

    def _homogeneous_to_cartesian(self, A):
        return A[:3, 3] / A[3, 3]

    def _forward_kin(self, angles):
        origin_p = -np.array([0, 0, 0])
        hip_p = -np.array([-0.086434, -0.012369, -0.171631])
        thigh_p = -np.array([0.004233, -0.032538, -0.171631])
        foot_p = -np.array([-0.27037, -0.147629, -0.050396])
        tip_p = -np.array([0.025909, -0.090243, -0.045146])

        origin = self._trans(origin_p)
        # origin -> hip
        R = self._rotx(angles[0])
        T = self._trans(hip_p - origin_p)
        origin_hip = origin @ T @ R

        # hip -> thigh
        R = self._roty(-angles[1])
        T = self._trans(thigh_p - hip_p)
        origin_thigh = origin_hip @ T @ R

        # thigh -> foot
        R = self._roty(angles[2])
        T = self._trans(foot_p - thigh_p)
        origin_sock = origin_thigh @ T @ R

        # foot -> tip
        T = self._trans(tip_p - foot_p)
        origin_tip = origin_sock @ T @ R

        return [
            sp.simplify(origin),
            sp.simplify(origin_hip),
            sp.simplify(origin_thigh),
            sp.simplify(origin_sock),
            sp.simplify(origin_tip),
        ]

    def fk(self, angles):
        return self._fk(angles.reshape(-1, 1))

    def ik(self, state_des, angles_init):
        opti = cs.Opti()
        angles = opti.variable(3)
        state = self._fk_cs(angles)[-1]
        obj = (state - state_des).T @ (state - state_des)
        opti.minimize(obj)
        opti.set_initial(angles, angles_init)
        opti.solver("ipopt", {"ipopt.print_level": 0, "print_time": 0})
        solution = opti.solve()

        return solution.value(angles), solution.value(state)
