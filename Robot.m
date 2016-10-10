clear;clc;
%% This lab makes use of rvcrobot toolkit by peter corke
%% Parameter Initilization 
a = [0,-0.3,-3,-2];
alpha = [0,pi/2,0,0];
d = [1,0,0,0];
Theta = [0,-pi/4,pi/4,0];

%% define robot link components
L0 = Link([0,d(1),a(1),alpha(1)],'modified');
L1 = Link([0,d(2),a(2),alpha(2)],'modified');
L2 = Link([0,d(3),a(3),alpha(3)],'modified');
L3 = Link([0,d(4),a(4),alpha(4)],'modified');

q = [0 -pi/4 pi/4 0];

bot = SerialLink([L0 L1 L2 L3], 'name', 'my robot');
T = bot.fkine([q(1) q(2) q(3) q(4)]);

T0 = transfer_m(a(1),alpha(1),d(1),Theta(1));
T1 = transfer_m(a(2),alpha(2),d(2),Theta(2));
T2 = transfer_m(a(3),alpha(3),d(3),Theta(3));
T3 = transfer_m(a(4),alpha(4),d(4),Theta(4));

T_EF2base = T0*T1*T2*T3;
EF = T_EF2base*[-0.5;0;0;1;];

%% Jacobian method 
P_d = [-0.5; 2; 5.5; 1];
theta_initial = [0; -pi/2; 0; pi/2]; 
for i=1:1000
    theta_delta{i}=[0;0;0;0];
end
    


for i = 1:1000
    T_0 = transfer_m(a(1),alpha(1),d(1),theta_initial(1));
    T_1 = transfer_m(a(2),alpha(2),d(2),theta_initial(2));
    T_2 = transfer_m(a(3),alpha(3),d(3),theta_initial(3));
    T_3 = transfer_m(a(4),alpha(4),d(4),theta_initial(4));

    P_pred = (T_0*T_1*T_2*T_3) * [-0.5;0;0;1;]

    P_er = [P_d - P_pred; 0; 0];
   
    T_jacb = bot.jacob0(theta_initial);
    T_jacb_pin = pinv(T_jacb);
    theta_delta{i} = T_jacb_pin*P_er;
    theta_initial = theta_initial + theta_delta{i};
%     if rem(i,4)==0
   %
%     end
end
bot.plot([theta_initial(1) theta_initial(2) theta_initial(3) theta_initial(4)]);