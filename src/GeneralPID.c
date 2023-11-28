#include "GeneralPID.h"

void PID_init(GeneralPID *self, float sampling_time, float kp, float ki, float kd)
{
  // setpoint for control routine
  self->setpoint = 0;
  // sampling time
  self->sampling_time = sampling_time;
  PID_setGains(self, kp, ki, kd);
  // variables used to store past erros and outputs
  self->error_n1 = 0;
  self->error_n2 = 0;
  self->output_n1 = 0;
  self->output_n2 = 0;
  // max output values
  self->max_output = +0xFFFF;
  self->min_output = -0xFFFF;
}

void PID_setGains(GeneralPID *self, float kp, float ki, float kd)
{
  // store PID gains inserted by the user
  self->KP = kp;
  self->KI = ki;
  self->KD = kd;
  // calculate discrete coefficients based on sampling
  float discrete_kp = self->KP;
  float discrete_ki = self->KI * self->sampling_time * 0.5;
  float discrete_kd = self->KD * self->sampling_time * 2;
  // store gains used to calculate the equivalent difference equation
  self->K1 = discrete_kp + discrete_ki + discrete_kd;
  self->K2 = (2 * discrete_ki) - (2 * discrete_kd);
  self->K3 = -discrete_kp + discrete_ki + discrete_kd;
  // store gains without considering integrator coeffiecient in case of output saturation
  self->K1_anti_windup = discrete_kp + discrete_ki * 0 + discrete_kd;
  self->K2_anti_windup = 2 * discrete_ki * 0 - 2 * discrete_kd;
  self->K3_anti_windup = -discrete_kp + discrete_ki * 0 + discrete_kd;
}

void PID_setMatlabGains(GeneralPID *self, float K1, float K2, float K3)
{
  // store gains used to calculate the equivalent difference equation
  self->K1 = K1;
  self->K2 = K2;
  self->K3 = K3;
}

void PID_setSetpoint(GeneralPID *self, float setpoint)
{
  self->setpoint = setpoint;
}

void PID_setBoundaries(GeneralPID *self, float max_output, float min_output)
{
  self->max_output = max_output;
  self->min_output = min_output;
}

float PID_execute(GeneralPID *self, float measurement)
{
  // error calculation
  float error = self->setpoint - measurement;

  // calculate differences equation
  float output = self->output_n1 + self->K1 * error + self->K2 * self->error_n1 + self->K3 * self->error_n2;

  if(output > self->max_output)
    output = self->max_output;
  if(output < self->min_output)
    output = self->min_output;

  // store last control values
  self->output_n2 = self->output_n1;
  self->output_n1 = output;
  self->error_n2 = self->error_n1;
  self->error_n1 = error;

  // return control action
  return output;
}