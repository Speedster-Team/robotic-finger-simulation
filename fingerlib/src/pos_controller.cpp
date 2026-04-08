#include "fingerlib/pos_controller.hpp"

PositionController::PositionController(double Kp, double Ki, double Kd)
: Kp_(Kp),
  Ki_(Ki),
  Kd_(Kd),
  Kff_(0.025),
  Kgvty_(0.025),
  Icntrl_ (true),
  Dcntrl_ (true),
  err_ (0.0),
  err_int_ (0.0),
  err_der_ (0.0),
  err_prev_ (0.0),
  i_clamp_val_ (5.0),
  u_clamp_val_ (5.0),
  feed_fwd_enable_ (true),
  ffwd_term_ (0.0),
  gvty_fwd_enable_ (true),
  gvty_term_ (0.5)
{
}

void PositionController::set_ffwd_control(bool enable)
{
    feed_fwd_enable_ = enable;
}

void PositionController::set_gvty_compensation(bool enable)
{
    gvty_fwd_enable_ = enable;
}

void PositionController::set_i_clamp_val(double clamp_val)
{
    i_clamp_val_ = clamp_val;
}

void PositionController::set_u_clamp_val(double clamp_val)
{
    u_clamp_val_ = clamp_val;
}

double PositionController::pump_controller(double setpoint, double actual, double next_cmd, float shaft_vel)
{

    err_ = setpoint - actual;
    if (feed_fwd_enable_) {ffwd_term_ = next_cmd;}
    if (Icntrl_) {err_int_ += err_;}

    if (Dcntrl_) { err_der_ = -(shaft_vel);} 
    err_prev_ = err_;

    // clamp err_int
    if (err_int_ > i_clamp_val_ && Icntrl_)
    {
        err_int_ = i_clamp_val_;
    } 
    else if (err_int_ < -i_clamp_val_ && Icntrl_)
    {
        err_int_ = -i_clamp_val_;
    }

    auto u = Kp_ * err_ + Ki_ * err_int_ + Kd_ * err_der_;

    // clamp command
    if (u > u_clamp_val_)
    {
        u = u_clamp_val_;
    } 
    else if (u < -u_clamp_val_)
    {
        u = -u_clamp_val_;
    }

    return u +  Kff_* gvty_term_ + Kff_ * ffwd_term_;
}