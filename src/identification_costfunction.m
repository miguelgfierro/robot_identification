function F = identification_costfunction(x,y)

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
filename = '../share/levantate_sinRs.csv';
[q1 q2 q3 Ts] = csv2TPtraj(filename);
T = length(q1)*Ts;
time = 0:Ts:T-Ts;
q = [q1 q2 q3];
[qd qdd] = derivate_data(q,time);

% ZMP computation
zmp_tp = calculate_zmp_TP(q,qd,qdd,M,L);

% Real zmp
zmp_left = load('../share/ZMP_left_leg');
zmp_right = load('../share/ZMP_right_leg');
zmp_real = (zmp_left(:,2) + zmp_right(:,2))/2*1e-3; % in m

% Constraint of torque
[tau_1 tau_2 tau_3] = ID_triple_pendulum_lagrange(M,L,q,qd,qdd); 
tmax = 9;
F_torque = 0;
for i=1:length(tau_1)
  if (abs(tau_1(i)) > tmax)
    F_torque = 1000;
  end
  if (abs(tau_2(i)) > tmax)
    F_torque = 1000;
  end  
  if (abs(tau_3(i)) > tmax)
    F_torque = 1000;
  end
end

% Constraint positivity
F_pos = 0;
for i=1:length(x)
  if x(i) < 0
    F_pos = F_pos + 100;
  end
end

% Constraint of lenght and weight
F_size = 0;
l_max = 0.70;
m_max = 9;
m_min = 0.5;
q_offset_min = -pi/2;
q_offset_max = pi/2;
if (l_1 + l_2 + l_3 > l_max)
  F_size = F_size + 100;
end
if (m_1 + m_2 + m_3 > m_max)
  F_size = F_size + 100;
end
if (m_1 < m_min || m_2 < m_min || m_2 < m_min)
  F_size = F_size + 50;
end
%if (q_offset < q_offset_min || q_offset > q_offset_max)
%  F_size = F_size + 50;
%end

% Cost function
F = sum(abs(zmp_real - zmp_tp)) + F_pos + F_size + F_torque;


