clear
clc

l2 = hypot(350, 50);
l3 = 360;
qarm_A = [400 400 150];
qarm_B = [400 400 110];
qarm_C = [450 200 300];
qarm_D = [500 0 350];
qarm_E = [600 -100 200];
qarm_F = [600 -100 135];
qarm_points = [qarm_A; qarm_B; qarm_C; qarm_D; qarm_E; qarm_F];
qarm_angles = zeros(6,3);
for i = 1:height(qarm_angles)
   x = qarm_points(i,1);
   y = qarm_points(i,2);
   z = qarm_points(i,3);
   qarm_angles(i, 1) = atan2d(y, x);
   alpha = atan2d(z - 140, hypot(x, y));
   beta = acosd((x^2 + y^2 + (z-140)^2 + l2^2 - l3^2)/(2 * l2 * hypot(hypot(x,y), z-140)));
   qarm_angles(i, 2) = -(alpha + beta);
   qarm_angles(i, 3) = acosd((x^2 + y^2 + (z-140)^2 - l2^2 - l3^2)/(2 * l2 * l3)) - 90;
end