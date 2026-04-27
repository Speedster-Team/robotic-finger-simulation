#include "fingerlib/sinusoid.hpp"
#include <armadillo>
#include <cmath>

Sinusoid::Sinusoid(const Transformer& transforms, int sampling_rate)
    : _transforms(transforms),
      _sampling_rate(sampling_rate)
{
}

std::vector<arma::vec> Sinusoid::generate_sinusoid(int joint, double amp, double freq, double v_shift){
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