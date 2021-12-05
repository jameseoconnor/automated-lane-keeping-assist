% Lateral Error Dynamics State Space Model
% Define static parameters
m = 2005;
Iz=4002;
lf=1.21; % assuming COG closer to front due to weight of engine
lr=1.61; % Total car track: 2.82
C_alpha = 56000; % Assuming cornering stiffnes on fromt and rear is equal



% 8.33, 0.02778, 0.08
% 13.89, 0.02723, 0.1
% 27.78, 0.01362, 0.15


% Define variable parameters
Vx=8.33;
ds=20; %look ahead distance


%State Space Model
A= [0 1 0 0;
    0 -4*C_alpha/(m*Vx) 4*C_alpha/m -(2*C_alpha*lf-2*C_alpha*lr)/(m*Vx);
    0 0 0 1;
    0 -(2*C_alpha*lf-2*C_alpha*lr)/(Iz*Vx) (2*C_alpha*lf-2*C_alpha*lr)/Iz -(2*C_alpha*lf^2+2*C_alpha*lr^2)/(Iz*Vx)];
B1=[0;2*C_alpha/m; 0; 2*C_alpha*lf/Iz];
B2=[0;-(2*C_alpha*lf-2*C_alpha*lr)/(m*Vx)-Vx; 0; -(2*C_alpha*lf^2+2*C_alpha*lr^2)/(Iz*Vx)];
C=[1 0 0 0; 0 0 1 0];
D=[0;0];

%Convert SS model to lookahead TF
[numP,denP]=ss2tf(A,B1,C,D) %converts SS model with steering angle input into two TF models in terms of output e1 (cross-track error) and e2 (heading error)

numP_y=ds*numP(1,:)+ds*numP(2,:) % Convert e1 (crosstrack error) and e2  into a single lookahead lateral position error y
P=tf(numP_y,denP)%TF lookahead output with steering angle input

[numG,denG]=ss2tf(A,B2,C,D)%conver SS model with desired yaw rate input into two TF models in terms of output e1 (cross-track error) and e2 (heading error)
numG_y=numG(1,:)+ds*numG(2,:) % Convert e1 (crosstrack error) and e2  into a single lookahead lateral position error y
G=tf(numG_y,denG)%TF for desired yaw rate input