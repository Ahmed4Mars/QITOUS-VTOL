clc
close all
s=tf('s');
% % Define the system G(s) = 10 / (s(s+2))
% num = [1];
% den = [1 5 0];
% G = tf(num, den);
% 
% % Bode plot of the open-loop system
% figure;
% bode(G);
% grid on;
% title('Open-Loop Bode Plot');
% % Desired specifications
% phi_desired = 50; % degrees
% wc_desired = 5; % rad/s (desired gain crossover)
% 
% % Calculate the required phase boost (add safety margin)
% phi_boost = phi_desired - 10 + 5; % Assuming current phase margin is 10°
% alpha = (1 - sind(phi_boost)) / (1 + sind(phi_boost));
% 
% % Calculate zero and pole of the lead controller
% wz = wc_desired / sqrt(alpha);
% wp = wc_desired * sqrt(alpha);
% 
% % Define the lead controller
% K = 1; % Adjust later for unity gain
% C = K * tf([1 wp], [1 wz]);
% 
% % Compensated system
% G_comp = C * G;
% 
% % Bode plot of compensated system
% figure;
% bode(G, G_comp);
% legend('Open-Loop', 'Compensated');
% grid on;
% title('Comparison of Open-Loop and Compensated Bode Plots');
% 
% figure;
% GSS = 10/(s^2+2*s);
% TT = feedback(GSS,1);
% step(TT)
% grid on
% 
% figure;
% bode(C, G);
% grid on
% legend('controller', 'plant');


GSS = 1/(s^2+5*s);
figure(1)
margin(GSS)
grid on
legend('Plant');
k1 = bode(GSS,10)
TT = feedback(GSS,1);
GCC = 1*(s+5.37)/(s+18.63);
k2 = bode(GCC,10)
K =1/k1/k2
%bode(GSS,GCC)
figure(2)
GCC = K*(s+5.37)/(s+18.63);
bode(GCC)
legend('Phase Lead Controller');
grid on
COMM = series(GSS,GCC)
figure(3)
margin(COMM)
grid on
legend('Compensated Plant');
% figure(4)
% T1 = feedback(GSS,1)
% step(T1)
% hold on
% grid on
% T2 = feedback(COMM,1)
% step(T2)
% grid on
% legend('Uncompensated Plant', 'Compensated Plant');

% GSS = 1/(s^2+5*s);
% controlSystemDesigner(GSS)
figure(4)
G= 1/(s^2+5*s);
GC = 208.1*(s+5.37)/(s+18.63);
GGC = series(GC,G);
bode(G,GC,GGC)
grid on
legend ('G','GC','GGC')

figure(5)
T1 = feedback(G,1)
step(T1)
hold on
T2 = feedback(GGC,1)
step(T2)
grid on
legend('Uncompensated Plant', 'Compensated Plant');