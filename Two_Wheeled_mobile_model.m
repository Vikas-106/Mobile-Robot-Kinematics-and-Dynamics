%% Kinematic model of a land based mobiel robot 
clear all;
clc;
close all;

%% Simulation Parameters 

dt = 0.1; % step size
ts = 100; % Total time for simulation
t = 0:dt:ts; % Time span of the simulation 

%% Initial Condition (starting coordinates)

x0 = 1; % x-coordinate
y0 = 1; % y-coordinate
psi0 = 0; % Angle of rotation 

eta0 = [x0;y0;psi0];

eta(:,1) = eta0;

%% robot configurations

a = 0.05 ; % Radius of wheel 
d = 0.2 ; % distance between wheel and center of the base 

%% loop starts here 

for i = 1:length(t)
    psi = eta(3,i);
    j_psi = [cos(psi), -sin(psi) , 0 ;  
             sin(psi), cos(psi) , 0 ;
             0, 0 , 1];

    %% input velocity for both the wheels

    omega_1 = 0.5; % Angular velocity of wheel 1 (left)
    omega_2 = 0.5; % Angular velocity of wheel 2  (right)

    omega = [omega_1 ;omega_2];
    W = [a/2 ,a/2 ;        
         0  ,0 ;               
         -a/(2*d) ,a/(2*d)];

    %% input velocity for macanum wheel (3 wheels with equal space )

    % omega_1 = 0; 
    % omega_2 = 0.5; 
    % omega_3 = -0.5;
    % 
    % omega = [omega_1 ;omega_2 ; omega_3];
    % 
    % W = (a/3)*[0 -sqrt(3) sqrt(3) ;
    %            2 -1 -1 ;
    %            1/d 1/d 1/d];

    %----------------------------------------------------

    zeta(:,i) = W*omega;

    eta_dot(:,i) = j_psi*zeta(:,i);

    eta(:,i+1) = eta(:,i)+dt*eta_dot(:,i);

end


%% Animation 

l = 0.7; % length of the robot
w = 2*d; % width of the robot 
r_coor = [-l/2, l/2 , l/2 , -l/2 ; 
           -w/2 , -w/2, w/2 , w/2];
figure;
for i = 1:5:length(t)
    psi = eta(3,i);
    r_mat = [cos(psi) , -sin(psi);  % Rotational matrix
             sin(psi) , cos(psi)];
    r_pos = r_mat * r_coor;
    fill(r_pos(1,:) + eta(1,i) , r_pos(2,:)+ eta(2,i),'g');
    hold on;
    plot(eta(1,1:i),eta(2,1:i));
    axis([x0-2 4 y0-2 4]);
    pause(0.01);
    hold off;
end
  
% Simulation ends 