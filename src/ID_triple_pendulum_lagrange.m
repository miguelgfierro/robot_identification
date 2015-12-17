% This function computes the inverse dynamics of a triple pendulum given
% the mass, length, position, velocity and acceleration.
% NOTE: Mass has to be in Kg, length in m, position in radians
% NOTE: The angles are taken over the vertical

function [tau_1 tau_2 tau_3] = ID_triple_pendulum_lagrange(M,L,q,qd,qdd)

% Masses and lengths
if (length(M) ~= 3 || length(L) ~= 3)
  error('Mass/length size mismatch');
end
m_1 = M(1);
m_2 = M(2);
m_3 = M(3);
l_1 = L(1);
l_2 = L(2);
l_3 = L(3);

% Trajectories
[n_trajectories vector_type] = min(size(q));
if (n_trajectories ~= 3)
  error('There has to be 3 DoF in the trajectory');
end
if(length(q) ~= length(qd) || length(q) ~= length(qdd))
  error('Size mismatch in qd/qdd');
end
if (vector_type == 1) % rows
  theta_1 = q(1,:);
  theta_2 = q(2,:);
  theta_3 = q(3,:);
  dtheta_1 = qd(1,:);
  dtheta_2 = qd(2,:);
  dtheta_3 = qd(3,:);
  d2theta_1 = qdd(1,:);
  d2theta_2 = qdd(2,:);
  d2theta_3 = qdd(3,:); 
else
  theta_1 = q(:,1);
  theta_2 = q(:,2);
  theta_3 = q(:,3);
  dtheta_1 = qd(:,1);
  dtheta_2 = qd(:,2);
  dtheta_3 = qd(:,3);
  d2theta_1 = qdd(:,1);
  d2theta_2 = qdd(:,2);
  d2theta_3 = qdd(:,3); 
end
  
% Euler - Lagrange equation:
% d/dt[dL/d(dai)] - dL/d(ai) = ti
% SC1 = dL/d(dai)           (SubChorizo1)
% SC2 = d/dt[dL/d(dai)]     (SubChorizo2)
% SC3 = dL/d(ai)            (SubChorizo3)
% C = SC2 - SC3; Chorizo = SubChorizo2 - SubChorizo3
g = 9.80665; %m/s²
tau_1 = d2theta_1.*l_1^2.*(m_1+m_2+m_3) + d2theta_2.*((m_2.*(2.*l_1.*l_2.*sin(theta_1).*sin(theta_2) + 2.*l_1.*l_2.*cos(theta_1).*cos(theta_2)))/2 + (m_3.*(2.*l_1.*l_2.*sin(theta_1).*sin(theta_2) + 2.*l_1.*l_2.*cos(theta_1).*cos(theta_2)))/2) - dtheta_2.*((m_2.*(2.*dtheta_2.*l_1.*l_2.*cos(theta_1).*sin(theta_2) - 2.*dtheta_2.*l_1.*l_2.*cos(theta_2).*sin(theta_1)))/2 + (m_3.*(2.*dtheta_2.*l_1.*l_2.*cos(theta_1).*sin(theta_2) - 2.*dtheta_2.*l_1.*l_2.*cos(theta_2).*sin(theta_1)))/2) - (dtheta_3.*m_3.*(2.*dtheta_3.*l_1.*l_3.*cos(theta_1).*sin(theta_3) - 2.*dtheta_3.*l_1.*l_3.*cos(theta_3).*sin(theta_1)))/2 + (d2theta_3.*m_3.*(2.*l_1.*l_3.*sin(theta_1).*sin(theta_3) + 2.*l_1.*l_3.*cos(theta_1).*cos(theta_3)))/2  - g.*l_1.*sin(theta_1).*(m_1+m_2+m_3);
tau_2 = d2theta_2.*l_2^2.*(m_2+m_3) + d2theta_1.*((m_2.*(2.*l_1.*l_2.*sin(theta_1).*sin(theta_2) + 2.*l_1.*l_2.*cos(theta_1).*cos(theta_2)))/2 + (m_3.*(2.*l_1.*l_2.*sin(theta_1).*sin(theta_2) + 2.*l_1.*l_2.*cos(theta_1).*cos(theta_2)))/2) + dtheta_1.*((m_2.*(2.*dtheta_1.*l_1.*l_2.*cos(theta_1).*sin(theta_2) - 2.*dtheta_1.*l_1.*l_2.*cos(theta_2).*sin(theta_1)))/2 + (m_3.*(2.*dtheta_1.*l_1.*l_2.*cos(theta_1).*sin(theta_2) - 2.*dtheta_1.*l_1.*l_2.*cos(theta_2).*sin(theta_1)))/2) - (dtheta_3.*m_3.*(2.*dtheta_3.*l_2.*l_3.*cos(theta_2).*sin(theta_3) - 2.*dtheta_3.*l_2.*l_3.*cos(theta_3).*sin(theta_2)))/2 + (d2theta_3.*m_3.*(2.*l_2.*l_3.*sin(theta_2).*sin(theta_3) + 2.*l_2.*l_3.*cos(theta_2).*cos(theta_3)))/2 - g.*l_2.*sin(theta_2).*(m_2+m_3);
tau_3 = d2theta_3.*l_3^2.*m_3 + (dtheta_1.*m_3.*(2.*dtheta_1.*l_1.*l_3.*cos(theta_1).*sin(theta_3) - 2.*dtheta_1.*l_1.*l_3.*cos(theta_3).*sin(theta_1)))/2 + (dtheta_2.*m_3.*(2.*dtheta_2.*l_2.*l_3.*cos(theta_2).*sin(theta_3) - 2.*dtheta_2.*l_2.*l_3.*cos(theta_3).*sin(theta_2)))/2 +  (d2theta_1.*m_3.*(2.*l_1.*l_3.*sin(theta_1).*sin(theta_3) + 2.*l_1.*l_3.*cos(theta_1).*cos(theta_3)))/2 + (d2theta_2.*m_3.*(2.*l_2.*l_3.*sin(theta_2).*sin(theta_3) + 2.*l_2.*l_3.*cos(theta_2).*cos(theta_3)))/2 - g.*l_3.*m_3.*sin(theta_3);

