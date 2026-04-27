#ifndef JOINT_TRAJECTORY_HPP
#define JOINT_TRAJECTORY_HPP

#include "armadillo"
#include "fingerlib/transformer.hpp"
#include <vector>  // needed for std::vector

/// \brief Basic sinusoid generator for testing 1 joint at a time
class JointTrajectory
{
public:
    /// \brief Initializer for sinusoid generator
    /// \param transforms - Transformer object for converting between joint and motor space
    /// \param sampling_rate - The rate at which to sample the sinusoid (in Hz)
    JointTrajectory(const Transformer& transforms, int sampling_rate, double ground_height);

    /// \brief Generate a sinusoidal trajectory for a specified joint + sine parameters
    /// \param joint - The index of the joint to generate the sinusoid for (0, 1, or 2)
    /// \param amp - The amplitude of the sinusoid (in radians)
    /// \param freq - The frequency of the sinusoid (in Hz)
    /// \param v_shift - The vertical shift of the sinusoid (in radians)
    /// \return A vector of motor positions corresponding to the generated sinusoidal trajectory
    std::vector<arma::vec> generate_sinusoid(int joint, double amp, double freq, double v_shift);

    /// \brief Generate a linear trajectory with trapezoidal time scaling
    /// \param start - joint space starting point
    /// \param end - joint space ending point
    /// \param v_max - max speed during operation
    /// \param a_max - max acceleration during operation  
    /// \param ground_height - relative height of the ground plane (should be negative)
    /// \return A vector of motor positions corresponding to the generated linear trajectory
    std::vector<arma::vec> generate_linear(arma::vec start, arma::vec end, double v_max, double a_max);

    /// \brief Generate a trajectory through joint space linear interpolation of cartesian waypoints
    /// \param waypoints - list of cartesian waypoints
    /// \param v_max - maximum speed during operation
    /// \param a_max - maximum acceleration during operation
    /// \return a vector of motor positions for the entire motion
    std::vector<arma::vec> generate_cartesian(std::vector<arma::vec> waypoints, double v_max, double a_max);

private:
    /// \brief The transformer object for converting between joint and motor space
    Transformer _transforms;

    /// \brief The sampling rate for the sinusoid (in Hz)
    int _sampling_rate;

    /// \brief height of the ground relative to fully extended pose (should be negative)
    double _ground_height;

    /// @brief Trapezoidal time scaling returning normalized position s(t) in [0, 1].
    ///
    /// Matches a trapezoidal velocity profile with peak velocity v_max and
    /// acceleration a_max. Automatically falls back to a triangular profile
    /// if the distance (normalized to 1) is too short to reach v_max.
    ///
    /// @param Tf     Total motion time
    /// @param t      Current time in [0, Tf]
    /// @param v_max  Maximum velocity (in normalized distance / unscaled time units)
    /// @param a_max  Maximum acceleration (in normalized distance / unscaled time^2 units)
    /// @param s      total motion length 
    /// @return       Normalized position s in [0, 1]
    double TrapezoidalTimeScaling(const double Tf, const double t,
                                const double v_max, const double a_max, const double s);

};

#endif // JOINT_TRAJECTORY