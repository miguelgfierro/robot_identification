% This function identify a triple pendulum system usign DE and ZMP data

warning('OFF')
clc
close all

[l_1 l_2 l_3 m_1 m_2 m_3] = initial_parameters;
gains_init = [l_1 l_2 l_3 m_1 m_2 m_3];

% DIFFERENTIAL EVOLUTION PARAMETERS
VTR = 1.e-2; 
if(exist('x')) % Si hay un resultado anterior lo tomamos
  gains = x;
else
  gains = gains_init;
end
D = length(gains); 
XVmin = round(gains - 1);
XVmax = round(gains + 1);
y=zeros(1,D); 
NP = 15;
itermax = 300; 
F = 0.8; 
CR = 0.8; 
strategy = 1;
refresh = 10; 

% DIFFERENTIAL EVOLUTION COMPUTATION
disp('Solving differential evolution optimization...')
[x,f,nf] = devec3('identification_costfunction',VTR,D,XVmin,XVmax,y,NP,itermax,F,CR,strategy,refresh)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             SOLUTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters of the triple pendulum
l_1 = x(1);
l_2 = x(2);
l_3 = x(3);
m_1 = x(4); 
m_2 = x(5); 
m_3 = x(6); 
%q_offset = x(7);
M = [m_1 m_2 m_3];
L = [l_1 l_2 l_3];

% Desired trajectory
filename = '../share/levantate_cabron03_sinRs.csv';
[q1 q2 q3 Ts] = csv2TPtraj(filename);
T = length(q1)*Ts;
time = 0:Ts:T-Ts;
q = [q1 q2 q3];
[qd qdd] = derivate_data(q,time);

% ZMP computation
zmp_tp = calculate_zmp_TP(q,qd,qdd,[m_1 m_2 m_3],[l_1 l_2 l_3]);

% Real zmp
zmp_left = load('../share/ZMP_left_leg');
zmp_right = load('../share/ZMP_right_leg');
zmp_real = (zmp_left(:,2) + zmp_right(:,2))/2*1e-3; % in m

% ZMP plot
plot_title = '';
%plot_title = 'Sagital robot ZMP ';
legend_axis = {'time (s)','ZMP (m)'};
legend_traj = {'ZMP_{TP}','ZMP_{real}'};
plot2traj(time,zmp_tp,zmp_real,plot_title, legend_traj,legend_axis)
hold on
zmp_robot_min = -0.040; %m
zmp_robot_max = 0.068;
plot(time,zmp_robot_max*ones(length(time)),'r-.',time,zmp_robot_min*ones(length(time)),'r-.')
hold off


% Torque plot
[tau_1 tau_2 tau_3] = ID_triple_pendulum_lagrange(M,L,q,qd,qdd); 
%plot_title = 'Robot toques';
plot_title = '';
legend_traj = {'\tau_1','\tau_2','\tau_3'};
legend_axis = {'time (s)','Toque (N.m)'};
plot3traj(time, tau_1, tau_2, tau_3, plot_title, legend_traj, legend_axis)
hold on
tmin = -9; %m
tmax = 9;
plot(time,tmin*ones(length(time)),'r-.',time,tmax*ones(length(time)),'r-.')
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% % Save results
% file_name = 'results_identification';
% cd ../demo
% fid = fopen(file_name, 'wt');
% for jj=1:length(x)
%   fprintf(fid,'%1.5f ',x(jj));
% end
% fclose(fid);
% cd ../src
% disp('Results saved to demo folder')


