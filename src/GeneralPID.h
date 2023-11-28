#ifndef GENERAL_PID_H // diretiva de inclusão condicional para evitar múltiplas inclusões do mesmo arquivo de cabeçalho
#define GENERAL_PID_H

typedef struct GeneralPID
{
  // setpoint
  float setpoint;
  // tempo de amostragem
  float sampling_time;
  // ganhos do PID
  float KP;
  float KI;
  float KD;
  // ganhos auxiliares
  float K1;
  float K2;
  float K3;
  // variaveis para armazenar os ultimos erros
  float error_n1;
  float error_n2;
  float output_n1;
  float output_n2;
  // determinacao dos limites de saida da malha
  float max_output;
  float min_output;
} GeneralPID;

// definicao das funcoes
void PID_init(GeneralPID *self, float sampling_time, float kp, float ki, float kd);
void PID_setGains(GeneralPID *self, float kp, float ki, float kd);
void PID_setMatlabGains(GeneralPID *self, float K1, float K2, float K3);
void PID_setSetpoint(GeneralPID *self, float setpoint);
void PID_setBoundaries(GeneralPID *self, float max_output, float min_output);
float PID_execute(GeneralPID *self, float measurement);

#endif // fecha a diretiva de inclusão condicional