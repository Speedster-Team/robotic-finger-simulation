"""Class leafsystem that computes tendon velocities."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class TendonFeedbackSystem(LeafSystem):
    """Leaf system computing tendon velocities."""

    def __init__(self, gear_ratio):
        """Create instance of TendonFeedbackSystem."""
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
        
        # Tendon stiffness vector
        self.k = np.array([0.1, 0.1, 0.1, 0.1])  # made up values for now

        # init size of input and output
        nq = 5  # five link positions
        nv = 5  # five link velocities
        nt = 4  # four tendon velocities
        nm = 1  # one motor velocity

        # init gear ratio
        self.gr = gear_ratio

        # declare input and output ports with functions
        self.joint_state_input_port = self.DeclareVectorInputPort(
            'finger_state', nq + nv)
        self.tendon_tension_input_port = self.DeclareVectorInputPort(
            'tendon_tension', nt)
        self.DeclareVectorOutputPort('tendon_velocity', nt, self._tendon_vel)
        self.DeclareVectorOutputPort('splay_velocity', nm, self._splay_vel)

        # structure of finger state
        # state x (size 10) = [q_splay, q_mcp_flex, q_pip1, q_dip1, q_pip2,
        #                      v_splay, v_mcp_flex, v_pip1, v_dip1, v_pip2]

    def _tendon_vel(self, context, output):
        """Convert joint state to joint velocity."""
        state = self.joint_state_input_port.Eval(context)
        tension = self.tendon_tension_input_port.Eval(context)

        # filter out splay, mcp, and pip
        vels = state[5:8]

        # calculate tension extension due to force
        stretch = tension * self.k  # do nothing with it for nowcb

        # transform to tendon space
        output.SetFromVector(self.Jt @ vels)

    def _splay_vel(self, context, output):
        """Scale splay velocity by gearbox and output."""
        state = self.joint_state_input_port.Eval(context)

        # output play velocity
        output.SetFromVector(np.array([state[5] / self.gr]))
