"""Class leafsystem that computes motor velocities."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class MotorFeedbackSystem(LeafSystem):
    """Leaf system computing motor velocities"""

    def __init__(self):
        """Create instance of MotorFeedbackSystem."""
        super().__init__()

       # define radii
        ra, rb, rc = 0.0025, 0.0025, 0.0025

        # define radius matrix
        self.Ra = np.array([[ra, 0, 0],
                            [0, rb, 0],
                            [0, 0, rc]])
        
        self.Ra_inv = np.linalg.pinv(self.Ra)
        
        nu = 3

        # declare input and output ports with functions
        self.tendon_velocity_input_port = self.DeclareVectorInputPort(
            'tendon_velocity', nu)
        self.tendon_position_input_port = self.DeclareVectorInputPort(
            'tendon_position', nu)
        self.DeclareVectorOutputPort('motor_velocity', nu, self._motor_vel)
        self.DeclareVectorOutputPort('motor_position', nu, self._motor_pos)

    def _motor_vel(self, context, output):
        """Convert tendon velocity to motor velocity."""
        tendon_vel = self.tendon_velocity_input_port.Eval(context)

        # compute motor vels
        vels = self.Ra_inv @ tendon_vel

        # transform to tendon space
        output.SetFromVector(vels)

    def _motor_pos(self, context, output):
        """Convert tendon position to motor velocity."""
        tendon_pos = self.tendon_position_input_port.Eval(context)

        # compute motor vels
        positions = self.Ra_inv @ tendon_pos

        # transform to tendon space
        output.SetFromVector(positions)
