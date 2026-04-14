"""Class leafsystem that transforms tendon force to joint torques."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class FingerPulleySystem(LeafSystem):
    """Leaf system converting motor torque tendon forces."""

    def __init__(self):
        """Create instance of FingerPulleySystem."""
        super().__init__()

        # define structure matrix

        # Pulley Radii [m]
        # Joint 0
        r1 = 8 * 0.001
        r2 = 8 * 0.001
        r3 = 4.5 * 0.001
        r4 = 4.5 * 0.001

        # Joint 1
        r5 = 8 * 0.001
        r6 = 8 * 0.001
        r7 = 4.5 * 0.001
        r8 = 4.5 * 0.001

        # Joint 2
        r9 = 9 * 0.001
        r10 = 9 * 0.001

        # Tendon Jacobian
        self.Jt = np.array([[-r1,  r5,  r9],
                            [r2, -r6, -r10],
                            [-r3,  r7, 0],
                            [r4, -r8, 0]])

        # init size of input and output
        nu = 3
        nt = 3
        nf = 3

        # declare input and output ports with functions
        self.input_port = self.DeclareVectorInputPort('tendon_tension', nu)
        self.input_port = self.DeclareVectorInputPort('motor_torque', nt)
        self.DeclareVectorOutputPort('joint_torque', nf, self._calc_tension)

    def _calc_tension(self, context, output):
        """Convert motor torque to tendon forces."""
        tension = self.input_port.Eval(context)
        output.SetFromVector(self.Jt @ tension)
