#include "fingerlib/transformer.hpp"
#include "modern_robotics/velocity_kinematics_and_statics.hpp"
#include "modern_robotics/forward_kinematics.hpp"

Transformer::Transformer(
    const arma::mat& Ra, 
    const arma::mat& structure, 
    const std::vector<arma::vec6>& screw_axes, 
    const arma::mat44& M,
    const std::vector<double>& four_bar_lengths)
    : _Ra(Ra), _Ra_inv(arma::pinv(_Ra)), 
      _structure(structure), _structure_inv(arma::pinv(_structure)), 
      _screw_axes(screw_axes), _M(M), 
      _4bar_lengths(four_bar_lengths)
{
}

arma::vec Transformer::joint_to_motor(const arma::vec& q_joint)
{
    arma::vec q_tendon = _structure.t() * q_joint;
    arma::vec q_motor = _Ra_inv * q_tendon;
    return q_motor;
}

arma::vec Transformer::motor_to_joint(const arma::vec& q_motor)
{
    arma::vec q_tendon = _Ra * q_motor;
    arma::vec q_joint = _structure_inv.t() * q_tendon;
    return q_joint;
}

arma::mat Transformer::get_jacobian_space(const arma::vec& q_joint)
{
    //TODO: come back to this and make sure its correct
    arma::mat J_full = mr::JacobianSpace(_screw_axes, q_joint);
    double dip_angle, speed_ratio;
    calculate_4bar_ratios(q_joint(2), dip_angle, speed_ratio);

    auto J = J_full.cols(0, 2); // only the first 3 columns are relevant for the 3 joints
    J.col(2) = J_full.col(2) + J_full.col(3)*speed_ratio; // add the contribution from the 4-bar linkage
    return J;
}

// the plan:
// Normally, IK uses a twist, but we will just use a position vector (x,y,z)
// Basically e = x_d - f(q) will be 3x3 instead of 6x6.
// This means that the Jacobian will also be 3x3 (using collapsed version), so it might be invertible?? will stick with pinv for now
// Will still use FK from modern robotics but just drop orientation
// might need to modify jacobian to make it coordinate not twist based..., should work fine using space frame though 
arma::vec Transformer::end_effector_to_joint(const arma::vec& q_end_effector){
  constexpr int max_iter = 20;
  constexpr double ev = 1e-3; // position error tolerance

  arma::vec thetalist(3, arma::fill::zeros); // initial guess for joint angles
  int i = 0;
  bool err = true;

  do {
    const arma::vec pos_error  = q_end_effector - joint_to_end_effector(thetalist).submat(0, 3, 2, 3); // position error (3x1 vector)
    const arma::mat Js_sub = get_jacobian_space(thetalist).submat(3, 0, 5, 2); // rows 3-5 for linear velocity, columns 0-2 for the 3 joints

    arma::vec dtheta;
    if (arma::cond(Js_sub) > 1e6) {
        // damped least squares
        std::cout << "Warning: Jacobian is near singular, using damped least squares" << std::endl;
        constexpr double lambda = 1e-2;
        dtheta = Js_sub.t() * arma::solve(Js_sub * Js_sub.t() + lambda * lambda * arma::eye(3, 3), pos_error);
    } else {
        dtheta = arma::pinv(Js_sub) * pos_error;
    }
    
    thetalist += dtheta;

    err = arma::norm(pos_error) > ev;
    ++i;
  } while (err && i < max_iter);

  // throw error if IK did not converge
  if (err) {
    throw std::runtime_error("IK did not converge");
  }

  return thetalist;
}

arma::mat44 Transformer::joint_to_end_effector(const arma::vec& q_joint){

    double pip_angle = q_joint(2);
    double dip_angle, speed_ratio;
    calculate_4bar_ratios(pip_angle, dip_angle, speed_ratio);

    arma::vec q_full = {q_joint(0), q_joint(1), q_joint(2), dip_angle};

    // compute the full forward kinematics using modern robotics
    arma::mat44 T = mr::FKinSpace(_M, _screw_axes, q_full);

    return T;
}

// from claude
void Transformer::calculate_4bar_ratios(
    double pip_angle,
    double& dip_angle,
    double& speed_ratio)
{
    const double L1 = _4bar_lengths[0];
    const double L2 = _4bar_lengths[1];
    const double L3 = _4bar_lengths[2];
    const double L4 = _4bar_lengths[3];
    
    // --- precompute reused trig terms ---
    const double t = (M_PI / 180.0) * pip_angle;
    const double s = std::sin(t);
    const double c = std::cos(t);
 
    // --- alpha/beta/gamma intermediates from half-angle substitution ---
    // K = (L1² + L2² - L3² + L4²) / (L2*L4)  [the "alpha" numerator factor]
    const double K  = (L1*L1 + L2*L2 - L3*L3 + L4*L4) / (L2 * L4);
 
    // Coefficients of the quadratic in tan(theta14/2):
    //   a_coef = cos(t) - L1/L2 - (L1/L4)*cos(t) + K/2
    //   b_coef = -2*sin(t)
    //   c_coef =  L1/L2 - (L1/L4)*cos(t) - cos(t) + K/2
    const double a_coef = c - L1/L2 - (L1/L4)*c + K/2.0;
    const double c_coef = L1/L2 - (L1/L4)*c - c + K/2.0;
    // b = -2s, discriminant = b²-4ac = 4s²-4*a_coef*c_coef
    const double disc = 4.0*s*s - 4.0*a_coef*c_coef;
 
    // half-angle numerator: sqrt(disc) + 2s  (the "+" branch)
    const double sqrtD   = std::sqrt(disc);
    const double numer   = sqrtD + 2.0*s;
    // denominator of tan half-angle: 2*a_coef (times 2 from b=-2s → (-b)=2s)
    const double denom   = 2.0 * (2.0*c - 2.0*L1/L2 - 2.0*(L1/L4)*c + K);
    // theta14 = 2*atan(numer/denom)
    const double theta14 = 2.0 * std::atan2(numer, denom);
 
    // coupler point coordinates (x3, y3)
    const double x3 = L1 - L2*c + L4*std::cos(theta14);
    const double y3 =     - L2*s + L4*std::sin(theta14);
 
    // theta3 (output angle, radians)
    dip_angle = std::atan2(y3, x3) - t;
 
    // --- derivative of theta14 w.r.t. theta2 ---
    // d(t)/d(theta2) = pi/180
    const double dt  = M_PI / 180.0;
 
    // d(disc)/d(theta2)  [chain rule through 4s²-4*a*c]
    // da/d(theta2) = (-s + (L1/L4)*s) * dt = s*(L1/L4 - 1)*dt
    // dc_coef/d(theta2) = (L1/L4*s + s)*dt = s*(1 + L1/L4)*dt  (note sign)
    const double da   = (-s + (L1/L4)*s) * dt;          // d(a_coef)/dt2
    const double dcc  = ((L1/L4)*s + s) * dt;            // d(c_coef)/dt2  (positive)
    const double d_disc = 8.0*s*c*dt - 4.0*(da*c_coef + a_coef*(-dcc));
    // Note: c_coef increases when theta2 increases → d(c_coef)/dt2 = +dcc
 
    // d(sqrtD)/d(theta2)
    const double d_sqrtD = d_disc / (2.0 * sqrtD);
 
    // d(numer)/d(theta2) = d_sqrtD + 2*c*dt
    const double d_numer = d_sqrtD + 2.0*c*dt;
 
    // d(denom)/d(theta2) = 2*(−2s*dt + 2*(L1/L4)*s*dt) = 4s*dt*(L1/L4 − 1)
    const double d_denom = 4.0*s*dt*(L1/L4 - 1.0);
 
    // d(atan(numer/denom))/d(theta2) via quotient rule
    const double ratio    = numer / denom;
    const double d_ratio  = (d_numer*denom - numer*d_denom) / (denom*denom);
    const double d_theta14 = 2.0 * d_ratio / (1.0 + ratio*ratio);
 
    // d(x3)/d(theta2), d(y3)/d(theta2)
    const double dx3 =  L2*s*dt - L4*std::sin(theta14)*d_theta14;
    const double dy3 = -L2*c*dt + L4*std::cos(theta14)*d_theta14;
 
    // d(atan2(y3,x3))/d(theta2)
    const double r2 = x3*x3 + y3*y3;
    const double d_atan2 = (x3*dy3 - y3*dx3) / r2;
 
    // theta3dot = d(theta3)/d(theta2)
    speed_ratio = d_atan2 - dt;
}