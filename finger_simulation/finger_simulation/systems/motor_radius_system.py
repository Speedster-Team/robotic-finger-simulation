"""Class describing leafsystem transforming motor torque to tendon tension."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class MotorTorqueToForceSystem(LeafSystem):
    """Leaf system converting motor torque tendon tension."""

    def __init__(self):
        """Create instance of MotorTorqueToForceSystem."""
        super().__init__()

        # define raidus matrix
        radius_matrix = np.eye(4) * 0.0025

        self.R_inv = radius_matrix ** -1  # invert
        nu = 3  # three acutators
        nf = 4  # four tendon tensions

        # declare input and output ports with functions
        self.input_port = self.DeclareVectorInputPort('motor_torque', nu)
        self.DeclareVectorOutputPort('tendon_tension', nf, self._calc_force)

    def _calc_force(self, context, output):
        """Convert motor torque to tendon forces."""
        torque = self.input_port.Eval(context)
        output.SetFromVector(self.R_inv @ torque)
