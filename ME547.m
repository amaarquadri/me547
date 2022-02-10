clc;
clear all;
close all;

% Declare the global variables for the robot
global d1;
d1 = 260;
global d4;
d4 = 20;
global d2;
d2 = hypot(290, d4);
global d3;
d3 = 78;



%%% Check the provided answers %%%
%   A1
disp("Expected: [467.40, 0, -21.71]");
disp(EndEffector(createDhTable(0, -45, -90, 135, 0))');

%   A2
disp("Expected: [245.56, 141.78, 54.44]");
disp(EndEffector(createDhTable(30, -90, -135, 135, 0))');

%   A3
disp("Expected: [411.04, -411.04, 34.57]");
disp(EndEffector(createDhTable(-45, -30, -45, 90, 0))');

%   A4
disp("Expected: [410.49, -191.41, 271.00]");
disp(EndEffector(createDhTable(-25, -70, -70, 110, 0))');
%%% Check the provided answers %%%

% theta_1 = sym('theta_1', 'real');
% theta_2 = sym('theta_2', 'real');
% theta_3 = sym('theta_3', 'real');
% theta_4 = sym('theta_4', 'real');
% theta_5 = sym('theta_5', 'real');
% 
% disp(EndEffector(createDhTable(theta_1, theta_2, theta_3, theta_4, theta_5))');


% Create the dh table given the angles of the robot
function dhTable = createDhTable(t1, t2, t3, t4, t5)
   % Get the robot parameters
   global d1;
   global d2;
   global d3;
   
   % Create the dh table
   dhTable = [  0       0       0       t1;
                0       -90     0       t2;
                d1      180     0       t3;
                d2      0       0       t4;
                0       90      0       t5;
                0       0       d3      0];
end 
     
% Compute the transformation matrix given the dh table values
function T = Transform(ai_1, ali_1, di, ti)
    % Compute a rundantly calculated value
    sc = sind(ti)*cosd(ali_1);
    cc = cosd(ti)*cosd(ali_1);
    ss = sind(ti)*sind(ali_1);
    cs = cosd(ti)*sind(ali_1);
    % Compute the transformation matrix
    T = [   cosd(ti)	-sind(ti)   0               ai_1;
            sc          cc         -sind(ali_1)    -di*sind(ali_1);
            ss          cs         cosd(ali_1)     di*cosd(ali_1);
            0           0           0               1];  
end

% Compute the end effector position given the dh table
function Epos = EndEffector(dhTable)
    % Compute the end effector position
    T_0_1 = Transform(dhTable(1, 1), dhTable(1, 2), dhTable(1, 3), dhTable(1, 4));
    T_1_2 = Transform(dhTable(2, 1), dhTable(2, 2), dhTable(2, 3), dhTable(2, 4));
    T_2_3 = Transform(dhTable(3, 1), dhTable(3, 2), dhTable(3, 3), dhTable(3, 4));
    T_3_4 = Transform(dhTable(4, 1), dhTable(4, 2), dhTable(4, 3), dhTable(4, 4));
    T_4_5 = Transform(dhTable(5, 1), dhTable(5, 2), dhTable(5, 3), dhTable(5, 4));
    T_5_6 = Transform(dhTable(6, 1), dhTable(6, 2), dhTable(6, 3), dhTable(6, 4));
    Epos =  T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6 * [0 0 0 1]';
end
