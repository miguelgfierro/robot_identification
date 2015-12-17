function [q1 q2 q3 Ts] = csv2TPtraj(filename)

Q = dlmread(filename,';');

encoder2rad = pi/180/209;
q1 = -Q(:,7).*encoder2rad;
q2 = (-Q(:,6) - Q(:,7)).*encoder2rad;
q3 = (Q(:,5) - Q(:,6) - Q(:,7)).*encoder2rad;
Ts = Q(1,1)*1e-3;








