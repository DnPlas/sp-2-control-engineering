clear
clc
% -------- CONSTANTS --------
% a = 2;
% b = 7;
% c = 21;
a = 3; 
b = 5; 
c = 1.27;
% -------- VARIABLES --------
r_t = a;
exponent = -0.003;
constant = 2*b;
e_extend = [(constant*exponent)/2 constant];
% -------- TRANSFER FUNCTION ---------
numerator = e_extend/2;
denominator = [1 (c/20) a/2];
G = tf(numerator, denominator)

% ############ CONTROLLERS ###########

% -------- PID PERPETUAL OSCILLATIONS--------
tu = 0.95;
ku = 8.4678;
kc = ku/1.7;
ti = tu/2;
td = tu/8;
ki = kc/ti;
kd = kc*td;
disp('PID - Perperual Oscillations Method:')
c_s = tf([kd kc ki] , [1 0])
% -------- PID DAMPED OSCILLATIONS --------
wn = sqrt(denominator(3));
delta = denominator(2)/(2*wn);
to = 0;
disp('PID -Damped Oscillations Method')
if delta < 0.21
    disp('This system cannot be solved by DAMPED OSCILATIONS METHOD')
else
    ko = to;
    kc = ku/1.7 ;
    ti = to/1.5;
    td = to/6;
    kd = kc*td;
    c_s = tf([kd kc ki] , [1 0])
end
% -------- PID USING SISOTOOL --------
%sisotool(G);
syms s;
compensator = 0.0055813; % This value was obtained 
mult = compensator*[1 34 240];
numerator = [mult(3) mult(2) mult(1)];
ki = numerator(3);
kd = numerator(1);
kc = numerator(2);
disp('PID - Sisotool Method:')
c_s = tf(numerator,[1 0])
% -------- LEAD COMPENSATOR --------
r_t = b;
a_1 = 1;
b_1 = 2;
ess = 0.03;
syms k;
eqn = 1/(1+(3.3*k)) == ess;
kp = double(vpasolve(eqn, k));
syms kk;
eqn = kk*(a_1/b_1) == kp;
k_k = double(vpasolve(eqn, kk));
numerator = k_k*[1 1];
denominator = [1 b_1];
c_s = tf(numerator, denominator)

% -------- LEAD-LAG COMPENSATOR --------
r_t = b;
syms k_1;
eqn_1 = 1/(1+(3.3*k_k*k_1)) == ess;
kp_1 = double(vpasolve(eqn_1, k_1));
b_2 = 0.1;
a_2 = kp_1*b_2;
c_s = tf([1 a_2],[1 b_2])