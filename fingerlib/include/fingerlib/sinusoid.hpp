#ifndef SINUSOID_HPP
#define SINUSOID_HPP

#include "armadillo"
#include "fingerlib/transformer.hpp"
#include <vector>  // needed for std::vector

/// \brief Basic sinusoid generator for testing 1 joint at a time
class Sinusoid
{
public:
    /// \brief Initializer for sinusoid generator
    /// \param transforms - Transformer object for converting between joint and motor space
    /// \param sampling_rate - The rate at which to sample the sinusoid (in Hz)
    Sinusoid(const Transformer& transforms, int sampling_rate);

    /// \brief Generate a sinusoidal trajectory for a specified joint + sine parameters
    /// \param joint - The index of the joint to generate the sinusoid for (0, 1, or 2)
    /// \param amp - The amplitude of the sinusoid (in radians)
    /// \param freq - The frequency of the sinusoid (in Hz)
    /// \param v_shift - The vertical shift of the sinusoid (in radians)
    std::vector<arma::vec> generate_sinusoid(int joint, double amp, int freq, double v_shift);

private:
    /// \brief The transformer object for converting between joint and motor space
    Transformer _transforms;

    /// \brief The sampling rate for the sinusoid (in Hz)
    int _sampling_rate;
};

#endif