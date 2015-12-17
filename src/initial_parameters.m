function [l_1 l_2 l_3 m_1 m_2 m_3 q_offset] = initial_parameters
% Distances of HOAP3 body in m(pag 6-5 of the manual)
ARM_LINK1=0.111;
ARM_LINK2=0.111;
ARM_LINK3=0.171;
LEG_LINK1=0.039;
LEG_LINK2=0.105;
LEG_LINK3=0.105;
LEG_LINK4=0.040;
BODY_LINK1=0.125;
BODY_LINK2=0.035;
HEAD_LINK1=0.103;
HEAD_LINK2=0.015;
WAIST_LINK1=0.055;
WAIST_LINK2=0.035;
front = 0.07; % lenght between eyes and top head
bagpack = 0.225; 

% Parameters of the triple pendulum
l_1 = LEG_LINK3 + LEG_LINK4;
l_2 = LEG_LINK2;
l_3 = WAIST_LINK1 + BODY_LINK1 + HEAD_LINK1 + front;
l_3 = l_3/3; % to put the cog at a 1/3 of the total lenght
m_1 = 1.345; % mass of both legs until the knee
m_2 = 1.5754; % mass from the knee to the waist
m_3 = 5.873; % mass of the rest of the body

q_offset = atan((bagpack/2)/(l_3));



