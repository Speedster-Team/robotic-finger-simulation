#ifndef SINUSOID_HPP
#define SINUSOID_HPP

#include "armadillo"
#include "fingerlib/transformer.hpp"
#include <vector>  // needed for std::vector

class Sinusoid
{
public:
    Sinusoid(const Transformer& transforms, int sampling_rate);

    std::vector<arma::vec> generate_sinusoid(int joint, double amp, int freq, double v_shift);

private:
    Transformer _transforms;
    int _sampling_rate;
};

#endif