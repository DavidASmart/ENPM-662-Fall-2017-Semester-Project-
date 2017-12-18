% -------------------------------------------------------------------------
% clear workspace
% -------------------------------------------------------------------------
clear
clc

numb = 6; % 6 blocks in stack

% -------------------------------------------------------------------------
% DH parameters
% -------------------------------------------------------------------------
d1 = 128;
a2 = 595;
a3 = 575;
d4 = 120;
d5 = 120;
d6 = 130;

% -------------------------------------------------------------------------
% Block Positions
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
% for all blocks in stack
for j = 1:numb
    Bz(j) = TB(3,4)+20*(numb-j);
end

% -------------------------------------------------------------------------
% Home-Position Angles
% -------------------------------------------------------------------------
theta1h = 0;
theta2h = 180;
theta3h = 180;
theta4h = 180;
theta5h = 180;
theta6h = 0;

% -------------------------------------------------------------------------
% Grab Angles
% -------------------------------------------------------------------------
theta1g = zeros(1,numb);
theta2g = zeros(1,numb);
theta3g = zeros(1,numb);
theta4g = zeros(1,numb);
theta5g = zeros(1,numb);
theta6g = zeros(1,numb);
for j = 1:numb
    theta6g(j) = 0;
    theta5g(j) = 90;
    theta1g(j) = Btheta;

    D = sqrt(Bx^2+By^2-d4^2)+d5;
    h = d6 + Bz(j);
    E = sqrt(D^2+(d1-h)^2);

    theta3g(j) = acosd((a2^2+a3^2-E^2)/(2*a2*a3)); % top
    psi = acosd((a2^2+E^2-a3^2)/(2*a2*E)); % close
    phi = acosd((a3^2+E^2-a2^2)/(2*a3*E)); % far

    if d1 > h
        theta2g(j) = 360-(psi + (90-atan2d(d1-h,D)));
        theta4g(j) = 360-(phi + atan2d(d1-h,D));
    else
        theta2g(j) = 360-(psi + (90+atan2d(h-d1,D)));
        theta4g(j) = 360-(phi - atan2d(h-d1,D));
    end
end

% -------------------------------------------------------------------------
% Place Angles
% -------------------------------------------------------------------------
theta1p = zeros(1,numb);
theta2p = zeros(1,numb);
theta3p = zeros(1,numb);
theta4p = zeros(1,numb);
theta5p = zeros(1,numb);
theta6p = zeros(1,numb);
for j = 1:numb
    theta6p(j) = 0;
    theta5p(j) = 90;
    theta1p(j) = theta1g(j)+90;
    
    theta2p(j) = theta2g(numb-j+1);
    theta3p(j) = theta3g(numb-j+1);
    theta4p(j) = theta4g(numb-j+1);
end

%--------------------------------------------------------------------------
% initialize time and angle commands
%--------------------------------------------------------------------------
t = 0:1:20*numb; t = t';
theta1 = zeros(size(t,1),1);
theta2 = zeros(size(t,1),1);
theta3 = zeros(size(t,1),1);
theta4 = zeros(size(t,1),1);
theta5 = zeros(size(t,1),1);
theta6 = zeros(size(t,1),1);
gripper = zeros(size(t,1),1);

for j = 1:numb % for all 6 blocks
    %--------------------------------------------------------------------------
    % pick up block
    %--------------------------------------------------------------------------
    % 4 seconds to move to block
    for i = 0+20*(j-1):4+20*(j-1)
        theta1(i+1) = theta1h + (theta1g(j)-theta1h)*(i-20*(j-1))/4;
        theta2(i+1) = theta2h + (theta2g(j)-theta2h)*(i-20*(j-1))/4;
        theta3(i+1) = theta3h + (theta3g(j)-theta3h)*(i-20*(j-1))/4;
        theta4(i+1) = theta4h + (theta4g(j)-theta4h)*(i-20*(j-1))/4;
        theta5(i+1) = theta5h + (theta5g(j)-theta5h)*(i-20*(j-1))/4;
        theta6(i+1) = theta6h + (theta6g(j)-theta6h)*(i-20*(j-1))/4;
        gripper(i+1) = 20;
    end
    % 2 seconds to grab block
    for i = 5+20*(j-1):6+20*(j-1)
        theta1(i+1) = theta1g(j);
        theta2(i+1) = theta2g(j);
        theta3(i+1) = theta3g(j);
        theta4(i+1) = theta4g(j);
        theta5(i+1) = theta5g(j);
        theta6(i+1) = theta6g(j);
        gripper(i+1) = 20 + (5-20)*(i-(20*(j-1)+4))/2;
    end

    %--------------------------------------------------------------------------
    % return to home position
    %--------------------------------------------------------------------------
    % 4 seconds to return
    for i = 7+20*(j-1):10+20*(j-1)
        theta1(i+1) = theta1g(j) - (theta1g(j)-theta1h)*(i-(20*(j-1)+6))/4;
        theta2(i+1) = theta2g(j) - (theta2g(j)-theta2h)*(i-(20*(j-1)+6))/4;
        theta3(i+1) = theta3g(j) - (theta3g(j)-theta3h)*(i-(20*(j-1)+6))/4;
        theta4(i+1) = theta4g(j) - (theta4g(j)-theta4h)*(i-(20*(j-1)+6))/4;
        theta5(i+1) = theta5g(j) - (theta5g(j)-theta5h)*(i-(20*(j-1)+6))/4;
        theta6(i+1) = theta6g(j) - (theta6g(j)-theta6h)*(i-(20*(j-1)+6))/4;
        gripper(i+1) = 5;
    end

    %--------------------------------------------------------------------------
    % place block
    %--------------------------------------------------------------------------
    % 4 seconds to move to drop spot
    for i = 11+20*(j-1):14+20*(j-1)
        theta1(i+1) = theta1h + (theta1p(j)-theta1h)*(i-(20*(j-1)+10))/4;
        theta2(i+1) = theta2h + (theta2p(j)-theta2h)*(i-(20*(j-1)+10))/4;
        theta3(i+1) = theta3h + (theta3p(j)-theta3h)*(i-(20*(j-1)+10))/4;
        theta4(i+1) = theta4h + (theta4p(j)-theta4h)*(i-(20*(j-1)+10))/4;
        theta5(i+1) = theta5h + (theta5p(j)-theta5h)*(i-(20*(j-1)+10))/4;
        theta6(i+1) = theta6h + (theta6p(j)-theta6h)*(i-(20*(j-1)+10))/4;
        gripper(i+1) = 5;
    end
    % 2 seconds to release block
    for i = 15+20*(j-1):16+20*(j-1)
        theta1(i+1) = theta1p(j);
        theta2(i+1) = theta2p(j);
        theta3(i+1) = theta3p(j);
        theta4(i+1) = theta4p(j);
        theta5(i+1) = theta5p(j);
        theta6(i+1) = theta6p(j);
        gripper(i+1) = 5 + (20-5)*(i-(20*(j-1)+14))/2;
    end

    %--------------------------------------------------------------------------
    % return to home position
    %--------------------------------------------------------------------------
    % 4 seconds to return
    for i = 17+20*(j-1):20+20*(j-1)
        theta1(i+1) = theta1p(j) - (theta1p(j)-theta1h)*(i-(20*(j-1)+16))/4;
        theta2(i+1) = theta2p(j) - (theta2p(j)-theta2h)*(i-(20*(j-1)+16))/4;
        theta3(i+1) = theta3p(j) - (theta3p(j)-theta3h)*(i-(20*(j-1)+16))/4;
        theta4(i+1) = theta4p(j) - (theta4p(j)-theta4h)*(i-(20*(j-1)+16))/4;
        theta5(i+1) = theta5p(j) - (theta5p(j)-theta5h)*(i-(20*(j-1)+16))/4;
        theta6(i+1) = theta6p(j) - (theta6p(j)-theta6h)*(i-(20*(j-1)+16))/4;
        gripper(i+1) = 20;
    end
end

%--------------------------------------------------------------------------
% Account for Imperfections
%--------------------------------------------------------------------------
% maximum gear backlash is ± 0.008° (toward ground)
% max encoder error is ± 0.001525° (assumed gaussian)
mu = 0;
sigma = 1/5;
obj = gmdistribution(mu,sigma);
for i = 0:size(t,1)
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
