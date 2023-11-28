%{

FILTRO LCR por Vitor Paese De Carli

O codigo tem o objetivo de demonstrar o projeto de um controlador do tipo
PID para controlar a tensao sobre o capacitor de um Filtro LC(R), onde a
resistencia R corresponde a resistencia de um carga generica.

%}

% limpando o ambiente
clc
clear all
close all

%% definicoes

% definicao de s
s = tf('s');
% frequencia de discretizacao do controlador
fc = 10e3;
tsc = 1/fc;

%% definicao do Filtro LCR em espa√ßo de estados

% parametros do sistema
L1 = 1e-3;
R1 = 0.5;
C1 = 44e-6;
R2 = 100;
% matriz de estados
A  = [-R1/L1    -1/L1;
      1/C1      -1/(R2*C1)];
% matriz de entrada
B  = [ 1/L1;
       0 ];
% matriz de saida
C = [0 1]; %escolhendo "ver" a tensao sobre o capacitor C1
% matriz de transicao direta
D  = 0;

%% extracao das funcoes de transferencia

%{
as funcoes de transferencia permitem usar o sisotool, a ferramenta acelera
o projeto quando o sistema possui apenas 1 entrada.
%}

%tensao do capacitor em relacao a tensao de entrada
[num, den]  = ss2tf(A,B,C,D);
VC_tf       = minreal(tf(num, den));

%% definicao do controlador PID
% controlador sintonizado no sisotool
PID  = 9.5e-6 * (s+10700)*(s+1000)*(1/s);
PIDd = minreal(c2d(PID, tsc, 'tustin'));

%% definicao da malha de controle
% malha aberta
VC_malha_aberta = PID * VC_tf;
% malha fechada
VC_malha_fechada = minreal(VC_malha_aberta / (1 + VC_malha_aberta)); 
% discretizacao da malha fechada
VC_malha_fechadad = minreal(c2d(VC_malha_fechada, tsc, 'tustin'));

% plot dos polos e zeros de malha fechada
figure(1)
pzmap(VC_malha_fechadad);
xlim([-1.1 +1.1]);
ylim([-1.1 +1.1]);

% resposta temporal da malha de controle
figure(2)
step(VC_malha_fechada);

%% "Prova real" para ver se nossos c·lculos estao certos...
% Pegar K1, K2 e K3 da funcao de transferencia do PID (em funcao de s)
kd = 9.5e-6;
kp = 0.1112;
ki = 101.7;
% Calcula os valores "discretizados" dos ganhos
kp1 = kp;
ki1 = ki * tsc * 1/2;
kd1 = kd * 2 * 1/tsc;
% Testa os valores ver se sao igual a K1, K2 e K3 do PIDd
k1 = kp1 + ki1 + kd1;
k2 = 2 * ki1 - 2 * kd1;
k3 = -kp1+ ki1 + kd1;