"""Class leafsystem that computes motor velocities."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class MotorFeedbackSystem(LeafSystem):
    """Leaf system computing motor velocities"""

    def __init__(self):
        """Create instance of MotorFeedbackSystem."""
        super().__init__()

        # define radii
        r1, r2 = 0.0025, 0.0025

        # define raidus matrix
        self.radius_matrix = np.array([[0, 1/r1, 0],
                                       [0, 0, -1/r1],
                                       [0, 1/r2, 0],
                                       [0, 0, -1/r2]])

        # init size of input and output
        nt = 4  # four tendon velocities
        nm = 1  # one motor velocity input

        # declare input and output ports with functions
        self.tendon_velocity_input_port = self.DeclareVectorInputPort(
            'tendon_velocity', nt)
        self.splay_velocity_input_port = self.DeclareVectorInputPort(
            'splay_velocity', nm)
        self.DeclareVectorOutputPort('motor_velocity', 3, self._motor_vel)

    def _motor_vel(self, context, output):
        """Convert tendon velocity to motor velocity."""
        tendon_vel = self.tendon_velocity_input_port.Eval(context)
        splay_vel = self.splay_velocity_input_port.Eval(context)

        # compute motor vels
        vels = self.radius_matrix.T @ tendon_vel

        # add on splay velocity
        vels[0] += splay_vel

        # transform to tendon space
        output.SetFromVector(vels)
