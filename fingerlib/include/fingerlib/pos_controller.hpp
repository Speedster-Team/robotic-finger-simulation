#ifndef POS_CONTROLLER_HPP
#define POS_CONTROLLER_HPP

#define GEAR_RATIO 1.0 // : 1

/// \brief Abstracted position controller for motor control
class PositionController
{
public:
  /// \brief Initializer for a PID controller using ffwd
  /// \param Kp - The proportional gain for a PID controller
  /// \param Ki - The integral gain for a PID controller
  /// \param Ki - The derivative gain for a PID controller
  PositionController(double Kp, double Ki, double Kd);

  /// \brief Pump controller
  /// \param setpoint - The set point position in radians
  /// \param actual - The actual motor shaft position in radians
  /// \param next_cmd - The next anticipated command for the motor in radians
  /// \param shaft_vel - The motor shaft velocity used in derivative control
  double pump_controller(double setpoint, double actual, double next_cmd, float shaft_vel);

  /// \brief Set the status of feed forward control in the class
  /// \param enable - Sets if feed forward term is used in control
  void set_ffwd_control(bool enable);

  /// \brief Set the status of gravity compensation control in the class
  /// \param enable - Sets if feed forward term is used in control
  void set_gvty_compensation(bool enable);

  /// \brief Set the value of the integral control error clamp
  /// \param clamp_val - The value that the integral error is clamped at
  void set_i_clamp_val(double clamp_val);

  /// \brief Set the value of the command clamp
  /// \param clamp_val - The value that the command output is clamped at
  void set_u_clamp_val(double clamp_val);

private:
  /// \brief Proportional gain
  double Kp_;
  /// \brief Integral gain
  double Ki_;
  /// \brief Derivative gain
  double Kd_;
  /// \brief Feed forward gain
  double Kff_;
  /// \brief Gravity compensation forward gain
  double Kgvty_;
  /// \brief Boolean enabling integral control
  bool Icntrl_;
  /// \brief Boolean enabling derivative control
  bool Dcntrl_;
  /// \brief Error
  double err_;
  /// \brief Integrated error
  double err_int_;
  /// \brief Derivative error
  double err_der_;
  /// \brief Previous error
  double err_prev_;
  /// \brief Integral error clamping value
  double i_clamp_val_;
  /// \brief Command clamping value
  double u_clamp_val_;
  /// \brief Boolean enabling feed forward control
  bool feed_fwd_enable_;
  /// \brief Feed forward compensation term
  double ffwd_term_;
  /// \brief Boolean enabling gravity compensation control
  bool gvty_fwd_enable_;
  /// \brief Gravity compensation term
  double gvty_term_;
};

#endif
