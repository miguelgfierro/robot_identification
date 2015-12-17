% This function calculates the ZMP of the triple pendulum given the
% trajectories of every link

function zmp = calculate_zmp_TP(q,qd,qdd,M,L)

% SYMBOLIC DERIVATIVES
% Derivate position, velocity and aceleration
%syms l_1 l_2 l_3 theta_1 theta_2 theta_3 dtheta_1 dtheta_2 dtheta_3 d2theta_1 d2theta_2 d2theta_3

% OBTANTION OF TRIPLE PENDULUM PARAMETERS
g = -9.80665; % m/s²
theta_1 = q(:,1);
theta_2 = q(:,2);
theta_3 = q(:,3);
dtheta_1 = qd(:,1);
dtheta_2 = qd(:,2);
dtheta_3 = qd(:,3);
d2theta_1 = qdd(:,1);
d2theta_2 = qdd(:,2);
d2theta_3 = qdd(:,3);
l_1 = L(1);
l_2 = L(2);
l_3 = L(3);
m_1 = M(1);
m_2 = M(2);
m_3 = M(3);

x_1 = l_1*sin(theta_1); 
z_1 = l_1*cos(theta_1);
x_2 = x_1 + l_2*sin(theta_2); 
z_2 = z_1 + l_2*cos(theta_2);
x_3 = x_2 + l_3*sin(theta_3);
z_3 = z_2 + l_3*cos(theta_3);

% xd_1 = fulldiff(x_1,theta_1);
% zd_1 = fulldiff(z_1,theta_1);
% xd_2 = fulldiff(x_2,{theta_1,theta_2});
% zd_2 = fulldiff(z_2,{theta_1,theta_2});
% xd_3 = fulldiff(x_3,{theta_1,theta_2,theta_3});
% zd_3 = fulldiff(z_3,{theta_1,theta_2,theta_3});

xd_1 = dtheta_1*l_1.*cos(theta_1);
zd_1 = -dtheta_1*l_1.*sin(theta_1);  
xd_2 = dtheta_1*l_1.*cos(theta_1) + dtheta_2*l_2.*cos(theta_2);
zd_2 = - dtheta_1*l_1.*sin(theta_1) - dtheta_2*l_2.*sin(theta_2);
xd_3 = dtheta_1*l_1.*cos(theta_1) + dtheta_2*l_2.*cos(theta_2) + dtheta_3*l_3.*cos(theta_3);
zd_3 = - dtheta_1*l_1.*sin(theta_1) - dtheta_2*l_2.*sin(theta_2) - dtheta_3*l_3.*sin(theta_3);

% xdd_1 = fulldiff(xd_1,theta_1);
% zdd_1 = fulldiff(zd_1,theta_1);
% xdd_2 = fulldiff(xd_2,{theta_1,theta_2});
% zdd_2 = fulldiff(zd_2,{theta_1,theta_2});
% xdd_3 = fulldiff(xd_3,{theta_1,theta_2,theta_3});
% zdd_3 = fulldiff(zd_3,{theta_1,theta_2,theta_3});

xdd_1 = d2theta_1*l_1.*cos(theta_1) - dtheta_1.^2*l_1.*sin(theta_1);
zdd_1 = - l_1*cos(theta_1).*dtheta_1.^2 - d2theta_1*l_1.*sin(theta_1);
xdd_2 = - l_1*sin(theta_1).*dtheta_1.^2 - l_2*sin(theta_2).*dtheta_2.^2 + d2theta_1*l_1.*cos(theta_1) + d2theta_2*l_2.*cos(theta_2);
zdd_2 = - l_1*cos(theta_1).*dtheta_1.^2 - l_2*cos(theta_2).*dtheta_2.^2 - d2theta_1*l_1.*sin(theta_1) - d2theta_2*l_2.*sin(theta_2);
xdd_3 = - l_1*sin(theta_1).*dtheta_1.^2 - l_2*sin(theta_2).*dtheta_2.^2 - l_3*sin(theta_3).*dtheta_3.^2 + d2theta_1*l_1.*cos(theta_1) + d2theta_2*l_2.*cos(theta_2) + d2theta_3*l_3.*cos(theta_3);
zdd_3 = - l_1*cos(theta_1).*dtheta_1.^2 - l_2*cos(theta_2).*dtheta_2.^2 - l_3*cos(theta_3).*dtheta_3.^2 - d2theta_1*l_1.*sin(theta_1) - d2theta_2*l_2.*sin(theta_2) - d2theta_3*l_3.*sin(theta_3);

% ZMP CALCULATION
% zmp1 = m_1*eval(x_1).*(eval(zdd_1)+g) + m_2*eval(x_2).*(eval(zdd_2)+g) + m_3*eval(x_3).*(eval(zdd_3)+g);
% zmp2 = m_1*eval(xdd_1).*eval(z_1);
% zmp3 = m_1*l_1^2*d2theta_1 + m_2*l_2^2*d2theta_2 + m_3*l_3^2*d2theta_3;
% zmp4 = m_1*(eval(zdd_1)+g) + m_2*(eval(zdd_2)+g) + m_3*(eval(zdd_3)+g);

zmp1 = m_1*x_1.*(zdd_1+g) + m_2*x_2.*(zdd_2+g) + m_3*x_3.*(zdd_3+g);
zmp2 = m_1*xdd_1.*z_1;
zmp3 = m_1*l_1^2*d2theta_1 + m_2*l_2^2*d2theta_2 + m_3*l_3^2*d2theta_3;
zmp4 = m_1*(zdd_1+g) + m_2*(zdd_2+g) + m_3*(zdd_3+g);
zmp = (zmp1 - zmp2 - zmp3)./zmp4; 




