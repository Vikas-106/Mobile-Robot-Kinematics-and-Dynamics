%% Kinematic model of a land based mobiel robot 
clear all;
clc;
close all;

%% Simulation Parameters 

dt = 0.1; % step size
ts = 10; % Total time for simulation
t = 0:dt:ts; % Time span of the simulation 

%% Initial Condition (starting coordinates)

x0 = 1; % x-coordinate
y0 = 1; % y-coordinate
psi0 = pi/3; % Angle of rotation 

eta0 = [x0;y0;psi0];

eta(:,1) = eta0;

%% loop starts here 

for i = 1:length(t)
    psi = eta(3,i);
    j_psi = [cos(psi), -sin(psi) , 0 ;  
             sin(psi), cos(psi) , 0 ;
             0, 0 , 1];
    %% input vel commands 
    u = 0.3; % longnitudunal velocity
    v = 0.1; % lateral velocity
    r = 0.1; % Rotation 
    zeta(:,i) = [u,v,r];

    eta_dot(:,i) = j_psi*zeta(:,i);

    eta(:,i+1) = eta(:,i)+dt*eta_dot(:,i);

end


%% Animation 

l = 0.5; % length of the robot
w = 0.4; % width of the robot 
r_coor = [-l/2, l/2 , l/2 , -l/2 ; 
           -w/2 , -w/2, w/2 , w/2];
figure;
for i = 1:length(t)
    psi = eta(3,i);
    r_mat = [cos(psi) , -sin(psi);  % Rotational matrix
             sin(psi) , cos(psi)];
    r_pos = r_mat * r_coor;
    fill(r_pos(1,:) + eta(1,i) , r_pos(2,:)+ eta(2,i),'g');
    hold on;
    plot(eta(1,1:i),eta(2,1:i));
    axis([x0-2 4 y0-2 4]);
    pause(0.1);
    hold off;
end
  