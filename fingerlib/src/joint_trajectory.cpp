#include "fingerlib/joint_trajectory.hpp"
#include <armadillo>
#include <cmath>

JointTrajectory::JointTrajectory(const Transformer& transforms, int sampling_rate, double ground_height)
    : _transforms(transforms),
      _sampling_rate(sampling_rate),
      _ground_height(ground_height)
{
}

std::vector<arma::vec> JointTrajectory::generate_sinusoid(int joint, double amp, double freq, double v_shift){
    int N = std::ceil((double)_sampling_rate / freq);

    std::vector<arma::vec> q_motor_list;
    q_motor_list.reserve(N);

    for(double t = 0; t < 1.0/freq; t += 1.0/_sampling_rate){

        double sine_value = amp * std::sin(2 * M_PI * freq * t) + v_shift;

        arma::vec q_joint(3, arma::fill::zeros); 

        if(joint >= 0 && joint <= 2){
            q_joint(joint) = sine_value;
        }

        q_motor_list.push_back(_transforms.joint_to_motor(q_joint));
    }

    //std::cout << "Generated " << q_motor_list.size() << " motor positions." << std::endl;

    return q_motor_list;
}

std::vector<arma::vec> JointTrajectory::generate_linear(arma::vec start, arma::vec end, double v_max, double a_max)
{
    double s = arma::norm(end-start);
    double Tf = s < (v_max*v_max)/a_max ? 2.0 * std::sqrt(s / a_max) : s / v_max + v_max / a_max;

    int N = std::ceil(Tf * (double)_sampling_rate);

    std::vector<arma::vec> q_motor_list;
    q_motor_list.reserve(N);

    for(double t = 0; t < Tf; t += 1.0/_sampling_rate){

        double st = TrapezoidalTimeScaling(Tf, t, v_max, a_max, s);
        auto q_joint = start*(1-st) + end*st;

        if(_transforms.joint_to_end_effector(q_joint)(2,3) < _ground_height){
            std::cout << _transforms.joint_to_end_effector(q_joint) << std::endl;
            throw std::runtime_error("Joint Trajectory goes into the ground");
        }

        q_motor_list.push_back(_transforms.joint_to_motor(q_joint));
    }


    return q_motor_list; 
}

std::vector<arma::vec> JointTrajectory::generate_cartesian(std::vector<arma::vec> waypoints, double v_max, double a_max)
{
    std::vector<arma::vec> trajectory;

    for(unsigned int i = 1; i < waypoints.size(); i++){
        auto start = _transforms.end_effector_to_joint(waypoints[i-1]);
        auto end = _transforms.end_effector_to_joint(waypoints[i]);

        auto linear_step = generate_linear(start, end, v_max, a_max);
        trajectory.insert(trajectory.end(), linear_step.begin(), linear_step.end());
    }

    return trajectory;
}

double JointTrajectory::TrapezoidalTimeScaling(const double Tf, const double t,
                               const double v_max, const double a_max, const double s)
{
  const bool is_triangular = ((v_max * v_max) / a_max > s);

  const double ta = is_triangular ? Tf / 2.0
                                  : v_max / a_max;

  double st = 0.0;

  if (t <= ta)
    st = 0.5 * a_max * t * t;
  else if (t <= Tf - ta)
    st = 0.5 * a_max * ta * ta + v_max * (t - ta);
  else
  {
    const double dt = Tf - t;
    st = s - 0.5 * a_max * dt * dt;
  }

  return st / s;  // normalize to [0, 1]
}