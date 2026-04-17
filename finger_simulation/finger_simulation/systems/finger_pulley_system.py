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
        self.Jt = np.array([[-r1,  r5,  r9],   # Tendon 4 (Green Tendon - PIP Extensor)
                            [r2, -r6, -r10],   # Tendon 1 (Red Tendon - PIP Flexor)
                            [-r3,  r7, 0],   # Tendon 2 (Purple Tendon - MCP Extensor)
                            [r4, -r8, 0]])  # Tendon 3 (Pink Tendon - MCP Flexor)

        # init size of input and output
        nu = 4  # four input tendon tensions
        ns = 1  # one splay motor torque
        nf = 3  # three actuated joints

        # declare input and output ports with functions
        self.tendon_input_port = self.DeclareVectorInputPort(
            'tendon_tension', nu)
        self.splay_input_port = self.DeclareVectorInputPort(
            'motor_splay_torque', ns)
        self.DeclareVectorOutputPort('joint_torque', nf, self._calc_tension)

    def _calc_tension(self, context, output):
        """Convert motor torque to tendon forces."""
        tension = self.tendon_input_port.Eval(context)
        splay_torque = self.splay_input_port.Eval(context)
        torques = self.Jt.T @ tension

        # combine induced torque from tendons on splay with capstan torque
        torques[0] += splay_torque

        output.SetFromVector(torques)
