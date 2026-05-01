"""Class describing leafsystem transforming motor torque to tendon tension."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class MotorTorqueToForceSystem(LeafSystem):
    """Leaf system converting motor torque tendon tension."""

    def __init__(self):
        """Create instance of MotorTorqueToForceSystem."""
        super().__init__()

        # define radii
        ra, rb, rc = 0.0025, 0.0025, 0.0025

        # define radius matrix
        self.Ra = np.array([[ra, 0, 0],
                            [0, rb, 0],
                            [0, 0, rc]])

        # self.R_inv = np.linalg.pinv(self.Ra)  # invert

        nu = 3

        # declare input and output ports with functions
        self.input_port = self.DeclareVectorInputPort('motor_torque', nu)
        self.DeclareVectorOutputPort('tendon_tension', nu, self._calc_force)

    def _calc_force(self, context, output):
        """Convert motor torque to tendon forces."""
        torque = self.input_port.Eval(context)
        output.SetFromVector(self.Ra @ torque)
