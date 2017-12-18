% -------------------------------------------------------------------------
% clear workspace
% -------------------------------------------------------------------------
clear
clc

% -------------------------------------------------------------------------
% DH parameters
% -------------------------------------------------------------------------
d1 = 128;
a2 = 595;
a3 = 575;
d4 = 120;
d5 = 120;
d6 = 120;

% -------------------------------------------------------------------------
% Block Position
% -------------------------------------------------------------------------
Btheta = 10;
r = 800;
TBt = [cosd(Btheta),-sind(Btheta),0,0;...
      sind(Btheta),cosd(Btheta),0,0;...
      0,0,1,0; 0,0,0,1];
TBr = [1,0,0,r;...
      0,1,0,0;...
      0,0,1,0; 0,0,0,1];
TBo = [1,0,0,0;...
      0,1,0,d4;...
      0,0,1,10; 0,0,0,1];
TB = TBt*TBr*TBo;
Bx = TB(1,4);
By = TB(2,4);
Bz = TB(3,4);

% -------------------------------------------------------------------------
% Initial Angles
% -------------------------------------------------------------------------
theta1i = 0;
theta2i = 180;
theta3i = 180;
theta4i = 180;
theta5i = 180;
theta6i = 0;

% -------------------------------------------------------------------------
% Final Angles
% -------------------------------------------------------------------------
theta6f = 0;
theta5f = 90;
theta1f = Btheta;
    
D = sqrt(Bx^2+By^2-d4^2)+d5;
h = d6 + Bz;
E = sqrt(D^2+(d1-h)^2);

theta3f = acosd((a2^2+a3^2-E^2)/(2*a2*a3)); % top
psi = acosd((a2^2+E^2-a3^2)/(2*a2*E)); % close
phi = acosd((a3^2+E^2-a2^2)/(2*a3*E)); % far

if d1 > h
    theta2f = 360-(psi + (90-atan2d(d1-h,D)));
    theta4f = 360-(phi + atan2d(d1-h,D));
else
    theta2f = 360-(psi + (90+atan2d(h-d1,D)));
    theta4f = 360-(phi - atan2d(h-d1,D));
end

%--------------------------------------------------------------------------
% initialize time and angle commands
%--------------------------------------------------------------------------
t = 0:1:20; t = t';
theta1 = zeros(size(t,1),1);
theta2 = zeros(size(t,1),1);
theta3 = zeros(size(t,1),1);
theta4 = zeros(size(t,1),1);
theta5 = zeros(size(t,1),1);
theta6 = zeros(size(t,1),1);
gripper = zeros(size(t,1),1);

%--------------------------------------------------------------------------
% pick up block
%--------------------------------------------------------------------------
% 4 seconds to move to block
for i = 0:4
    theta1(i+1) = theta1i + (theta1f-theta1i)*i/4;
    theta2(i+1) = theta2i + (theta2f-theta2i)*i/4;
    theta3(i+1) = theta3i + (theta3f-theta3i)*i/4;
    theta4(i+1) = theta4i + (theta4f-theta4i)*i/4;
    theta5(i+1) = theta5i + (theta5f-theta5i)*i/4;
    theta6(i+1) = theta6i + (theta6f-theta6i)*i/4;
    gripper(i+1) = 20;
end
% 2 seconds to grab block
for i = 5:6
    theta1(i+1) = theta1f;
    theta2(i+1) = theta2f;
    theta3(i+1) = theta3f;
    theta4(i+1) = theta4f;
    theta5(i+1) = theta5f;
    theta6(i+1) = theta6f;
    gripper(i+1) = 20 + (10-20)*(i-4)/2;
end

%--------------------------------------------------------------------------
% return to home position
%--------------------------------------------------------------------------
% 4 seconds to return
for i = 7:10
    theta1(i+1) = theta1f - (theta1f-theta1i)*(i-6)/4;
    theta2(i+1) = theta2f - (theta2f-theta2i)*(i-6)/4;
    theta3(i+1) = theta3f - (theta3f-theta3i)*(i-6)/4;
    theta4(i+1) = theta4f - (theta4f-theta4i)*(i-6)/4;
    theta5(i+1) = theta5f - (theta5f-theta5i)*(i-6)/4;
    theta6(i+1) = theta6f - (theta6f-theta6i)*(i-6)/4;
    gripper(i+1) = 10;
end

%--------------------------------------------------------------------------
% place block
%--------------------------------------------------------------------------
theta1f = theta1f + 90;
% 4 seconds to move to drop spot
for i = 11:14
    theta1(i+1) = theta1i + (theta1f-theta1i)*(i-10)/4;
    theta2(i+1) = theta2i + (theta2f-theta2i)*(i-10)/4;
    theta3(i+1) = theta3i + (theta3f-theta3i)*(i-10)/4;
    theta4(i+1) = theta4i + (theta4f-theta4i)*(i-10)/4;
    theta5(i+1) = theta5i + (theta5f-theta5i)*(i-10)/4;
    theta6(i+1) = theta6i + (theta6f-theta6i)*(i-10)/4;
    gripper(i+1) = 10;
end
% 2 seconds to release block
for i = 15:16
    theta1(i+1) = theta1f;
    theta2(i+1) = theta2f;
    theta3(i+1) = theta3f;
    theta4(i+1) = theta4f;
    theta5(i+1) = theta5f;
    theta6(i+1) = theta6f;
    gripper(i+1) = 10 + (20-10)*(i-14)/2;
end

%--------------------------------------------------------------------------
% return to home position
%--------------------------------------------------------------------------
% 4 seconds to return
for i = 17:20
    theta1(i+1) = theta1f - (theta1f-theta1i)*(i-16)/4;
    theta2(i+1) = theta2f - (theta2f-theta2i)*(i-16)/4;
    theta3(i+1) = theta3f - (theta3f-theta3i)*(i-16)/4;
    theta4(i+1) = theta4f - (theta4f-theta4i)*(i-16)/4;
    theta5(i+1) = theta5f - (theta5f-theta5i)*(i-16)/4;
    theta6(i+1) = theta6f - (theta6f-theta6i)*(i-16)/4;
    gripper(i+1) = 20;
end
%--------------------------------------------------------------------------
% Account for Imperfections
%--------------------------------------------------------------------------
% maximum gear backlash is ± 0.008° (toward ground)
% max encoder error is ± 0.001525° (assumed gaussian)
mu = 0;
sigma = 1/5;
obj = gmdistribution(mu,sigma);
for i = 0:20
    theta1(i+1) = theta1(i+1)+random(obj)*0.001525;
    theta2(i+1) = theta2(i+1)+random(obj)*0.001525+0.008;
    theta3(i+1) = theta3(i+1)+random(obj)*0.001525+0.008;
    theta4(i+1) = theta4(i+1)+random(obj)*0.001525-0.008;
    theta5(i+1) = theta5(i+1)+random(obj)*0.001525;
    theta6(i+1) = theta6(i+1)+random(obj)*0.001525;
    gripper(i+1) = gripper(i+1)+random(obj)*0.001525;
end

%--------------------------------------------------------------------------
% convert to timeseries
%--------------------------------------------------------------------------
theta1t = timeseries(theta1,t,'Name','theta1');
theta2t = timeseries(theta2,t,'Name','theta2');
theta3t = timeseries(theta3,t,'Name','theta3');
theta4t = timeseries(theta4,t,'Name','theta4');
theta5t = timeseries(theta5,t,'Name','theta5');
theta6t = timeseries(theta6,t,'Name','theta6');
gripper = gripper*1/1000;
grippert = timeseries(gripper,t,'Name','gripper');

%--------------------------------------------------------------------------
% Save Trajectory
%--------------------------------------------------------------------------
save('theta1.mat','theta1t','-v7.3')
save('theta2.mat','theta2t','-v7.3')
save('theta3.mat','theta3t','-v7.3')
save('theta4.mat','theta4t','-v7.3')
save('theta5.mat','theta5t','-v7.3')
save('theta6.mat','theta6t','-v7.3')
save('gripper.mat','grippert','-v7.3')
