#ifndef TRANSFORMS_HPP
#define TRANSFORMS_HPP

#include "armadillo"

// notes
// 0 is fully extended

class Transformer
{
public:

Transformer(const arma::mat& Ra, const arma::mat& structure, const std::vector<arma::vec6>& screw_axes, const arma::vec& four_bar_lengths);

arma::vec joint_to_motor(const arma::vec& q_joint);
arma::vec motor_to_joint(const arma::vec& q_motor);

arma::vec joint_to_end_effector(const arma::vec& q_joint);
arma::vec end_effector_to_joint(const arma::vec& q_end_effector);
arma::vec get_jacobian_space(const arma::vec& q_joint);

private:

// constant things 
const arma::mat _Ra;
const arma::mat _Ra_inv;
const arma::mat _structure;
const arma::mat _structure_inv;
const std::vector<arma::vec6> _screw_axes;
const arma::vec _4bar_lengths;

// functions
void calculate_4bar_ratios(double pip_angle, double& dip_angle, double& speed_ratio);

};

#endif

