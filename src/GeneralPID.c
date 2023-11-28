#include "GeneralPID.h"

void PID_init(GeneralPID *self, float sampling_time, float kp, float ki, float kd)
{
  // setpoint
  self->setpoint = 0;
  // tempo de amostragem
  self->sampling_time = sampling_time;
  PID_setGains(self, kp, ki, kd);
  // variaveis para armazenar valores passados
  self->error_n1 = 0;
  self->error_n2 = 0;
  self->output_n1 = 0;
  self->output_n2 = 0;
  // valores maximos de saida da mlja
  self->max_output = +0xFFFF;
  self->min_output = -0xFFFF;
}

void PID_setGains(GeneralPID *self, float kp, float ki, float kd)
{
  // armazena os valores de ganhos inseridos pelo usuario
  self->KP = kp;
  self->KI = ki;
  self->KD = kd;
  // calcula os coeficientes "discretizados"
  float discrete_kp = self->KP;
  float discrete_ki = self->KI * self->sampling_time * 0.5;
  float discrete_kd = self->KD * self->sampling_time * 2;
  // armazena os ganhos correspondentes a equacao de diferencas
  self->K1 = discrete_kp + discrete_ki + discrete_kd;
  self->K2 = (2 * discrete_ki) - (2 * discrete_kd);
  self->K3 = -discrete_kp + discrete_ki + discrete_kd;
}

void PID_setMatlabGains(GeneralPID *self, float K1, float K2, float K3)
{
  // armazena os ganhos de forma direita, feito para utilizar com c2d(func, ts, 'tustin')
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
  // definicao dos limites de saida da malha
  self->max_output = max_output;
  self->min_output = min_output;
}

float PID_execute(GeneralPID *self, float measurement)
{
  // calcula do erro
  float error = self->setpoint - measurement;

  // calcula a equacao de diferenças
  float output = self->output_n1 + self->K1 * error + self->K2 * self->error_n1 + self->K3 * self->error_n2;

  // estabeleces as limitacoes de saida
  if(output > self->max_output)
    output = self->max_output;
  if(output < self->min_output)
    output = self->min_output;

  // armazena os valores anteriores de erro e saida
  self->output_n2 = self->output_n1;
  self->output_n1 = output;
  self->error_n2 = self->error_n1;
  self->error_n1 = error;

  // retorna o valor da acao de controle
  return output;
}