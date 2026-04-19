"""Class describing leafsystem transforming motor torque to tendon tension."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class MotorTorqueToForceSystem(LeafSystem):
    """Leaf system converting motor torque tendon tension."""

    def __init__(self):
        """Create instance of MotorTorqueToForceSystem."""
        super().__init__()

        # define radii
        r1, r2 = 0.0025, 0.0025

        # define raidus matrix
        self.radius_matrix = np.array([[0, 1/r1, 0],  # mcp
                                       [0, -1/r1, 0],  # mcp
                                       [0, 0, 1/r2],  # pip
                                       [0, 0, -1/r2]])  # pip

        # self.R_inv = np.linalg.pinv(radius_matrix)  # invert
        nu = 3  # three acutators
        nf = 4  # four tendon tensions

        # declare input and output ports with functions
        self.input_port = self.DeclareVectorInputPort('motor_torque', nu)
        self.DeclareVectorOutputPort('tendon_tension', nf, self._calc_force)

    def _calc_force(self, context, output):
        """Convert motor torque to tendon forces."""
        torque = self.input_port.Eval(context)
        tensions = self.radius_matrix @ torque
        mask = tensions < 0
        tensions[mask] = 0
        output.SetFromVector(tensions)
