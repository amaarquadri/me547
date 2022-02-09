clc;
clear all;
close all;

% Declare the global variables for the robot
global d1;
d1 = 260;
global d2;
d2 = 290;
global d3;
d3 = 78;
global d4;
d4 = 20;

%%% Check the provided answers %%%
%   A1
disp(EndEffector(createDhTable(0, -45, -90, 135, 0)));
disp("Expected: [467.40, 0, -21.71]");
disp(" ")
disp(" ")

%   A2
disp(EndEffector(createDhTable(30, -90, -135, 135, 0)));
disp("Expected: [245.56, 141.78, 54.44]");
disp(" ")
disp(" ")

%   A3
disp(EndEffector(createDhTable(-45, -30, -45, 90, 0)));
disp("Expected: [411.04, -411.04, 34.57]");
disp(" ")
disp(" ")

%   A4
disp(EndEffector(createDhTable(-25, -70, -70, 110, 0)));
disp("Expected: [410.49, -191.41, 271.00]");
disp(" ")
disp(" ")
%%% Check the provided answers %%%




% Create the dh table given the angles of the robot
function dhTable = createDhTable(t1, t2, t3, t4, t5)
   % Get the robot parameters
   global d1;
   global d2;
   global d3;
   global d4;
   
   % Create the dh table
   dhTable = [  0       0       0       t1;
                0       90      0       90 + t2;
                d1      180     0       90;
                d2      0       0       t3 - 90;
                0       90      d4      t4;
                0       t5      d3      0];
end
     
% Compute the transformation matrix given the dh table values
function T = Transform(ai_1, ali_1, di, ti)
    % Compute the transformation matrix
    T = [   cosd(ti)	-sind(ti)   0               ai_1;
            sind(ti)*cosd(ali_1)	cosd(ti)*cosd(ali_1)	-sind(ali_1)    -di*sind(ali_1);
            sind(ti)*sind(ali_1)	cosd(ti)*sind(ali_1)	cosd(ali_1)     di*cosd(ali_1);
            0           0           0               1];  
end

% Compute the end effector position given the dh table
function Epos = EndEffector(dhTable)
    % Compute the end effector position
    Epos =  Transform(dhTable(1, 1), dhTable(1, 2), dhTable(1, 3), dhTable(1, 4)) * ...
            Transform(dhTable(2, 1), dhTable(2, 2), dhTable(2, 3), dhTable(2, 4)) * ...
            Transform(dhTable(3, 1), dhTable(3, 2), dhTable(3, 3), dhTable(3, 4)) * ...
            Transform(dhTable(4, 1), dhTable(4, 2), dhTable(4, 3), dhTable(4, 4)) * ...
            Transform(dhTable(5, 1), dhTable(5, 2), dhTable(5, 3), dhTable(5, 4)) * ...
            Transform(dhTable(6, 1), dhTable(6, 2), dhTable(6, 3), dhTable(6, 4)) * ...
            [0; 0; 0; 1];         
end
