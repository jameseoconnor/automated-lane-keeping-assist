% Lateral Error Dynamics State Space Model
% Define static parameters
m = 2005;
Iz=4002;
lf=1.41; % Don't know so assuming COG in centre of car
lr=1.41;
C_alpha = 70000; % Assuming cornering stiffnes on fromt and rear is equal 

% Define variable parameters
Vx = 100; 
ds=30; %look ahead distance


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

numP_y=numP(1,:)+ds*numP(2,:) % Convert e1 (crosstrack error) and e2  into a single lookahead lateral position error y
P=tf(numP_y,denP)%TF lookahead output with steering angle input

[numG,denG]=ss2tf(A,B2,C,D)%conver SS model with desired yaw rate input into two TF models in terms of output e1 (cross-track error) and e2 (heading error)
numG_y=numG(1,:)+ds*numG(2,:) % Convert e1 (crosstrack error) and e2  into a single lookahead lateral position error y
G=tf(numG_y,denG)%TF for desired yaw rate input




