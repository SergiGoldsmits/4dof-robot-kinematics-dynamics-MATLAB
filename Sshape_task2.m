clear; close all; clc;

%% --- ROBOT PARAMETERS ---

% Link lengths [m]
l0 = 200e-3; 
l1 = 329e-3;
l2 = 311.50e-3;
l3 = 106e-3;
r0 = 60e-3;
r123 = 40e-3;

% Joint limits
q1_min = deg2rad(-180);    q1_max = deg2rad(180);
q2_min = deg2rad(-180);    q2_max = deg2rad(180);
q3_min = deg2rad(-180);    q3_max = deg2rad(180);
q4_min = deg2rad(-180);    q4_max = deg2rad(180);

joint_limits = [q1_min q1_max;
                q2_min q2_max;
                q3_min q3_max;
                q4_min q4_max];

% Center of mass lengths [m]
g1 = l1/2; 
g2 = l2/2; 
g3 = l3/2; 

%masses [kg]
m0 = 17;
m1 = 10;
m2 = 9;
m3 = 3;
m_payload=0.5; 

% Inertias [kg*m^2]
I0_xy = (1/4)*m0*r0^2 + (1/12)*m0*l0^2;
I0_z  = (1/2)*m0*r0^2;
I1_yz = (1/4)*m1*r123^2 + (1/12)*m1*l1^2; 
I1_x  = (1/2)*m1*r123^2;
I2_yz = (1/4)*m2*r123^2 + (1/12)*m2*l2^2; 
I2_x  = (1/2)*m2*r123^2;
I3_yz = (1/4)*m3*r123^2 + (1/12)*m3*l3^2; 
I3_x  = (1/2)*m3*r123^2;


%% --- MOTORS PARAMETERS ---
motor.A_lim = 600; % max motor acc [rad/s^2]
motor.V_lim = 600; % max motor vel [rad/s]
motor.D_lim = 600; % max motor dec[rad/s^2]

% Servo motor HG-KN 23 (B)J (brake+resolver)
Cn =0.64; % Nominal Torque          [Nm]
mot_inertia = 0.247*10^-4;      %[kg*m^2]  
rid = 1/100;
alpha = Cn^2/mot_inertia; % Accererating Factor
%% --- JOINTS PARAMETERS  ---
motor.A = motor.A_lim * rid;   % max joint acc[rad/s^2]
motor.V = motor.V_lim * rid;   % max joint vel [rad/s]
motor.D = motor.D_lim * rid;   % max joint dec[rad/s^2]

L = [l0; l1; l2; l3; g1; g2; g3];

% Masses diagonal matrix
M=diag([m_payload, m_payload,m_payload, m3, m3, m3, I3_yz,...
    m2, m2, m2, I2_yz, m1, m1, m1, I1_yz, m0, m0, m0, ...
    I0_z]);  

%% External forces vector

Fse = zeros(19,1);    % extended version of external forces
Fse(2)  = -m_payload*9.81; % Fz payload
Fse(5)  = -m3*9.81;        % Fz m3
Fse(9)  = -m2*9.81;        % Fz m2
Fse(13) = -m1*9.81;        % Fz m1
Fse(17) = -m0*9.81;        % Fz m0

nJ  = 4;    % n. of Joints

%% --- Waypoints on the S-space---

A = [0.25;  0.00; 0.05;  deg2rad(0)];    
D = [0.35;  0.00; 0.20;  deg2rad(0)];   
E = [0.50;  0.15; 0.20;  deg2rad(0)];



%% Sampling time
dt  = 0.1;      

%% Calculate minimum actuation time
% Inverse kinematics for A, D, E (elbow-up)
QA = ROBOTinv_4R(A, L, -1);
QD = ROBOTinv_4R(D, L, -1);
QE = ROBOTinv_4R(E, L, -1);

dq_AD = abs(QD - QA);   % A -> D 
dq_DE = abs(QE - QD);   % D -> E 
dq_ED = abs(QD - QE);   % E -> D 
dq_DA = abs(QA - QD);   % D -> A 

dQ=[dq_AD;dq_DE;dq_ED;dq_DA];
dQ_abs= abs(dQ);

rise_time=zeros(4,size(dQ,2));
for i=1:size(dQ,2)
    rise_time(1,i)=min_rise_time(motor,dQ_abs(1,i));
    rise_time(2,i)=min_rise_time(motor,dQ_abs(2,i));
    rise_time(3,i)=min_rise_time(motor,dQ_abs(3,i));
    rise_time(4,i)=min_rise_time(motor,dQ_abs(4,i));

end

T_min_act= max(rise_time);
disp('Min Actuation Time:');
disp(T_min_act);

    
%% --- S-Shape time configuration ---
T_AD = T_min_act * 1.1;   % A->D line  [s]
T_DE = T_min_act * 1.1;   % D->E arch  [s]
T_ED = T_min_act * 1.1;   % E->D arch  [s]
T_DA = T_min_act * 1.1;   % D->A line  [s]

t_AD = 0:dt:T_AD;
t_DE = 0:dt:T_DE;
t_ED = 0:dt:T_ED;
t_DA = 0:dt:T_DA;

N_AD = length(t_AD);
N_DE = length(t_DE);
N_ED = length(t_ED);
N_DA = length(t_DA);

% S-Shape profile wrt time
ta_AD = T_AD/3; tb_AD = 2*T_AD/3; tc_AD = T_AD;
ta_DE = T_DE/3; tb_DE = 2*T_DE/3; tc_DE = T_DE;
ta_ED = T_ED/3; tb_ED = 2*T_ED/3; tc_ED = T_ED;
ta_DA = T_DA/3; tb_DA = 2*T_DA/3; tc_DA = T_DA;


% Global time & S-vectors inizialization
S_all     = [];    % [4 x Nt]
Sdot_all  = [];
Sddot_all = [];
t_all     = [];

%% ===== 1) A -> D  (LINE)=====

S_start = A;
S_end   = D;
dS      = S_end - S_start;

for k = 1:N_AD
    t = t_AD(k);

    rx   = Sshape(t, S_start(1), dS(1), ta_AD, tb_AD, tc_AD);
    ry   = Sshape(t, S_start(2), dS(2), ta_AD, tb_AD, tc_AD);
    rz   = Sshape(t, S_start(3), dS(3), ta_AD, tb_AD, tc_AD);
    rtet = Sshape(t, S_start(4), dS(4), ta_AD, tb_AD, tc_AD);

    S_now     = [rx.pos;  ry.pos;  rz.pos;  rtet.pos];
    Sdot_now  = [rx.vel;  ry.vel;  rz.vel;  rtet.vel];
    Sddot_now = [rx.acc;  ry.acc;  rz.acc;  rtet.acc];

    S_all     = [S_all,     S_now];
    Sdot_all  = [Sdot_all,  Sdot_now];
    Sddot_all = [Sddot_all, Sddot_now];
end
t_all = [t_all, t_AD];

%% ===== 2) D -> E (CIRCLE IN XY PLANE) =====

% Select the centre of the circle in XY plane
Cx = 0.50;    % [m]
Cy = 0.00;    % [m]

% Compute the Radius
R = sqrt( (D(1)-Cx)^2 + (D(2)-Cy)^2 );   

% Polar coordinates
phi0 = atan2(D(2)-Cy, D(1)-Cx);      % phi wrt D    
phi1 = atan2(E(2)-Cy, E(1)-Cx);      % phi wrt E 
dphi_DE = phi1 - phi0;               % total phi excursion

% Z and Teta (Linear)
dz_DE    = E(3) - D(3);
dthet_DE = E(4) - D(4);

% S-Shape application to D->E
for k = 2:N_DE
    t = t_DE(k);

    rphi = Sshape(t, phi0, dphi_DE, ta_DE, tb_DE, tc_DE);

    phi    = rphi.pos;
    phi_p  = rphi.vel;
    phi_pp = rphi.acc;

    % Back to XY coordinates
    x  = Cx + R*cos(phi);
    y  = Cy + R*sin(phi);

    x_p  = -R*sin(phi)*phi_p;
    y_p  =  R*cos(phi)*phi_p;

    x_pp = -R*cos(phi)*phi_p^2 - R*sin(phi)*phi_pp;
    y_pp = -R*sin(phi)*phi_p^2 + R*cos(phi)*phi_pp;

    rz   = Sshape(t, D(3), dz_DE,    ta_DE, tb_DE, tc_DE);
    rtet = Sshape(t, D(4), dthet_DE, ta_DE, tb_DE, tc_DE);

    S_now     = [x;      y;        rz.pos;   rtet.pos];
    Sdot_now  = [x_p;    y_p;      rz.vel;   rtet.vel];
    Sddot_now = [x_pp;   y_pp;     rz.acc;   rtet.acc];

    S_all     = [S_all,     S_now];
    Sdot_all  = [Sdot_all,  Sdot_now];
    Sddot_all = [Sddot_all, Sddot_now];
end
% t_all = [t_all, t_DE + T_AD];    
t_all = [t_all, t_DE(2:end) + t_all(end)];


%% ===== 3) E-> D (CIRCLE IN XY PLANE) =====

dphi_ED = phi0 - phi1;

dz_ED    = D(3) - E(3);
dthet_ED = D(4) - E(4);

for k = 2:N_ED
    t = t_ED(k);

    rphi = Sshape(t, phi1, dphi_ED, ta_ED, tb_ED, tc_ED);

    phi    = rphi.pos;
    phi_p  = rphi.vel;
    phi_pp = rphi.acc;

    x  = Cx + R*cos(phi);
    y  = Cy + R*sin(phi);

    x_p  = -R*sin(phi)*phi_p;
    y_p  =  R*cos(phi)*phi_p;

    x_pp = -R*cos(phi)*phi_p^2 - R*sin(phi)*phi_pp;
    y_pp = -R*sin(phi)*phi_p^2 + R*cos(phi)*phi_pp;

    rz   = Sshape(t, E(3), dz_ED,    ta_ED, tb_ED, tc_ED);
    rtet = Sshape(t, E(4), dthet_ED, ta_ED, tb_ED, tc_ED);

    S_now     = [x;      y;        rz.pos;   rtet.pos];
    Sdot_now  = [x_p;    y_p;      rz.vel;   rtet.vel];
    Sddot_now = [x_pp;   y_pp;     rz.acc;   rtet.acc];

    S_all     = [S_all,     S_now];
    Sdot_all  = [Sdot_all,  Sdot_now];
    Sddot_all = [Sddot_all, Sddot_now];
end
% t_all = [t_all, t_ED + T_AD + T_DE];  
t_all = [t_all, t_ED(2:end) + t_all(end)];


%% ===== 4) D -> A (LINEAR) =====

S_start = D;
S_end   = A;
dS      = S_end - S_start;

for k = 2:N_DA
    t = t_DA(k);

    rx   = Sshape(t, S_start(1), dS(1), ta_DA, tb_DA, tc_DA);
    ry   = Sshape(t, S_start(2), dS(2), ta_DA, tb_DA, tc_DA);
    rz   = Sshape(t, S_start(3), dS(3), ta_DA, tb_DA, tc_DA);
    rtet = Sshape(t, S_start(4), dS(4), ta_DA, tb_DA, tc_DA);

    S_now     = [rx.pos;  ry.pos;  rz.pos;  rtet.pos];
    Sdot_now  = [rx.vel;  ry.vel;  rz.vel;  rtet.vel];
    Sddot_now = [rx.acc;  ry.acc;  rz.acc;  rtet.acc];

    S_all     = [S_all,     S_now];
    Sdot_all  = [Sdot_all,  Sdot_now];
    Sddot_all = [Sddot_all, Sddot_now];
end
% t_all = [t_all, t_DA + T_AD + 2*T_DE];     
t_all = [t_all, t_DA(2:end) + t_all(end)];


Nt    = length(t_all);


%% ======= INVERSE KINEMATICS & DYNAMICS =========

q_all   = zeros(nJ, Nt);
qd_all  = zeros(nJ, Nt);
qdd_all = zeros(nJ, Nt);

q1t_save  = zeros(1,Nt);
q2t_save  = zeros(1,Nt);
q3t_save  = zeros(1,Nt);
q4t_save  = zeros(1,Nt);
qt_save = zeros(4,Nt);
q1tm_save = zeros(1,Nt);
q2tm_save = zeros(1,Nt);
q3tm_save = zeros(1,Nt);
q4tm_save = zeros(1,Nt);

for k = 1:Nt

    S  = [ S_all(1,k);
           S_all(2,k);
           S_all(3,k);  
           S_all(4,k) ];

    Sp  = Sdot_all(:,k); 
    Spp = Sddot_all(:,k);  

    % INVERSE KINEMATCS
    Q = ROBOTinv_4R(S, L, -1);       % Elbow-up

        % --- Joint limits check at time step k ---
    if any(Q < joint_limits(:,1) | Q > joint_limits(:,2))
        error("ERROR: configuration at time step %d violates joint limits.", k);
    end

    J    = ROBOTjac_4R(Q, L);
    Qp   = J \ Sp;
    Jdot = ROBOTjacP_4R(Q, Qp, L);
    Qpp  = J \ (Spp - Jdot*Qp);

    q_all(:,k)   = Q;
    qd_all(:,k)  = Qp;
    qdd_all(:,k) = Qpp;

    % Dynamic Jacobian
    J_din     = ROBOTjacdin_4R(Q, L);
    J_dot_din = ROBOTjacPdin_4R(Q, Qp, L);
    
    % Extended accelerations
    Sdpp = J_dot_din*Qp + J_din*Qpp;

    % Inertial Forces
    Fsi  = -M*Sdpp;

    % Total Forces
    Fs   = Fse + Fsi;

    % Joint Torques
    Fcq  = -J_din.' * Fs;

    q1t_save(k) = Fcq(1);
    q2t_save(k) = Fcq(2);
    q3t_save(k) = Fcq(3);
    q4t_save(k) = Fcq(4);
    qt_save(:,k) = Fcq;
    
    app_1(k) = q1t_save(k) * qdd_all(1,k);
    app_2(k) = q2t_save(k) * qdd_all(2,k);
    app_3(k) = q3t_save(k) * qdd_all(3,k);
    app_4(k) = q4t_save(k) * qdd_all(4,k);
    app(:,k) = Fcq .* qdd_all(:,k);
 

    % Motor Torques
    q1tm_save(k) = q1t_save(k)*rid + mot_inertia*qdd_all(1,k)*(1/rid);
    q2tm_save(k) = q2t_save(k)*rid + mot_inertia*qdd_all(2,k)*(1/rid);
    q3tm_save(k) = q3t_save(k)*rid + mot_inertia*qdd_all(3,k)*(1/rid);
    q4tm_save(k) = q4t_save(k)*rid + mot_inertia*qdd_all(4,k)*(1/rid);
end

% Load Factor check
beta = zeros(1,nJ);      
rid_opt = zeros(1,nJ);   
rid_max = zeros(1,nJ);
rid_min = zeros(1,nJ);

for p=1:nJ

    wrq(p) = rms(qd_all(p,:));
    dwrq(p) = rms(qdd_all(p,:));
    Crsq(p) = rms(qt_save(p,:));
    dwCm(p) = mean(app(p,:));

    rid_opt = sqrt(mot_inertia*dwrq(p)/Crsq(p));

    beta(p) = ((rid/sqrt(mot_inertia)) * Crsq(p) - (sqrt(mot_inertia)/rid) * wrq(p))^2 + 2*(Crsq(p)*dwrq(p) + dwCm(p)); 

    rid_max(p) = (sqrt(mot_inertia)*(sqrt(alpha-beta(p)+4*dwrq(p)*Crsq(p))...
            + sqrt(alpha - beta(p))))/(2*Crsq(p));

    rid_min(p) = (sqrt(mot_inertia)*(sqrt(alpha-beta(p)+4*dwrq(p)*Crsq(p))...
            - sqrt(alpha - beta(p))))/(2*Crsq(p));
    
end

figure; hold on; grid on;

joints = 1:4;

% Punti beta
scatter(joints, beta, 80, 'b', 'filled');

% Threshold alpha
yline(alpha, 'r--', 'LineWidth', 1.8);

xlabel('Joint index');
ylabel('\beta');
title('Motor Sizing Check: \beta values vs. \alpha threshold');
xticks(joints);

legend('\beta_j', '\alpha threshold');

if all(beta < alpha)
    fprintf("All joints satisfy the motor sizing condition α > β.\n");
else
    fprintf("WARNING: Some joints violate α > β α.\n");
    idx = find(beta >= alpha);
    fprintf("Joints which do not respect the condition: ");
    fprintf("%d ", idx);
    fprintf("\n");
end

figure; hold on; grid on;

joints = 1:4;

% Plot rid_min
scatter(joints, rid_min, 90, 'g', 'filled');

% Plot rid_max
scatter(joints, rid_max, 90, 'm', 'filled');

% rid
yline(rid, 'b-', 'LineWidth', 1.6);

xlabel('Joint index');
ylabel('Reduction ratio r_{id}');
title('Motor Reduction Ratio Check');
xticks(joints);

legend('r_{id}^{min}', 'r_{id}^{max}', 'chosen r_{id}', ...
       'Location', 'best');




%% --- Kinematics quality index k^{-1} along trajectory ---

nt = numel(t_all); 
Kinv = zeros(1, nt);

for k = 1:nt
    Qk = q_all(:,k);

    J = ROBOTjac_4R(Qk, L);

    Jv = J(1:3,:);

    s = svd(Jv);
    smax = max(s);
    smin = min(s);

    if smax < 1e-8
        
        Kinv(k) = 0;
    else
        Kinv(k) = smin/smax;
    end
end

% Results
fprintf('\n=== Quality Index results ===\n');
fprintf('k^{-1}_min = %.4f\n', min(Kinv));    % min K^-1
fprintf('k^{-1}_max = %.4f\n', max(Kinv));    % max K^-1

figure;
plot(t_all, Kinv, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('k^{-1}');
title('Quality Index vs Time');

% K^-1 threshold
thr_primary = 0.2;   
yline(thr_primary, '--r', 'Primary threshold');
legend('k^{-1}(t)', 'thr_{primary}');


figure; % Pos
sgtitle('Joint Positions');

for j = 1:4
    subplot(2,2,j); hold on; grid on;
    plot(t_all, rad2deg(q_all(j,:)), 'LineWidth', 1.2);
    xlabel('t [s]');
    ylabel('q [deg]');
    title(sprintf('Joint %d - Position', j));
end

figure; % Vel
sgtitle('Joint Velocities');

for j = 1:4
    subplot(2,2,j); hold on; grid on;
    plot(t_all, qd_all(j,:), 'LineWidth', 1.2);
    xlabel('t [s]');
    ylabel('q\_dot [rad/s]');
    title(sprintf('Joint %d - Velocity', j));
end

figure; % Acc
sgtitle('Joint Accelerations');

for j = 1:4
    subplot(2,2,j); hold on; grid on;
    plot(t_all, qdd_all(j,:), 'LineWidth', 1.2);
    xlabel('t [s]');
    ylabel('q\_ddot [rad/s^2]');
    title(sprintf('Joint %d - Acceleration', j));
end

figure;
subplot(2,2,1); plot(t_all,q1t_save); xlabel('Time [s]'); ylabel('T\_Joint [Nm]'); grid on; title('q1 torque');
subplot(2,2,2); plot(t_all,q2t_save); xlabel('Time [s]'); ylabel('T\_Joint [Nm]'); grid on; title('q2 torque');
subplot(2,2,3); plot(t_all,q3t_save); xlabel('Time [s]'); ylabel('T\_Joint [Nm]'); grid on; title('q3 torque');
subplot(2,2,4); plot(t_all,q4t_save); xlabel('Time [s]'); ylabel('T\_Joint [Nm]'); grid on; title('q4 torque');

figure;
subplot(2,2,1); plot(t_all,q1tm_save); xlabel('Time [s]'); ylabel('T\_Motor [Nm]'); grid on; title('q1 motor torque');
subplot(2,2,2); plot(t_all,q2tm_save); xlabel('Time [s]'); ylabel('T\_Motor [Nm]'); grid on; title('q2 motor torque');
subplot(2,2,3); plot(t_all,q3tm_save); xlabel('Time [s]'); ylabel('T\_Motor [Nm]'); grid on; title('q3 motor torque');
subplot(2,2,4); plot(t_all,q4tm_save); xlabel('Time [s]'); ylabel('T\_Motor [Nm]'); grid on; title('q4 motor torque');

%% --- TRAJECTORY IN S-SPACE ---

figure;
hold on; grid on;

plot3(S_all(1,:), S_all(2,:), S_all(3,:), 'b', 'LineWidth', 1.5);

% waypoint A-D-E-D-A
W = [A, D, E, D, A];    
plot3(W(1,:), W(2,:), W(3,:), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Traiettoria spazio operativo 3D');
axis equal;
view(3);


% %% ANIMATION 

f1 = figure;                      
f1.WindowStyle='normal';
hold on; grid on; view(3);

ll1=line('XData', [0 0 0], ...
         'YData', [0 0 0], ...
         'ZData', [0 0 0], ...
         'linestyle', '-','linewidth',2,'color','r',...
         'marker','o','markersize',6,'markerfacecolor','k');

trail = plot3(NaN,NaN,NaN,'LineWidth',1.5);
trail_X = []; trail_Y = []; trail_Z = [];

for k = 1:size(q_all,2)
    PlotRobot2(q_all(:,k), L, l0, 'r', f1, ll1);

    Se = ROBOTdir_4R(q_all(:,k), L);   % [X;Y;Z;theta]
    Xe = Se(1)*1e3;
    Ye = Se(2)*1e3;
    Ze = Se(3)*1e3;

    trail_X(end+1) = Xe;
    trail_Y(end+1) = Ye;
    trail_Z(end+1) = Ze;

    set(trail, 'XData', trail_X, 'YData', trail_Y, 'ZData', trail_Z);
    drawnow;
end

%% --Simscape utilities--
tt=t_all;

% Signal Editor

% Joint 1
ww1=qd_all(1,:);
signal1 = Simulink.SimulationData.Signal;
signal1.Name = 'vel1_task2';
signal1.Values = timeseries(ww1*(1/rid), tt);
signal1.Values = setinterpmethod(signal1.Values,'linear'); 
ds1_task2 = Simulink.SimulationData.Dataset;
ds1_task2 = addElement(ds1_task2, signal1, 'velocity');
save('vel1_task2.mat', 'ds1_task2');

% Joint 2
ww2=qd_all(2,:);
signal2 = Simulink.SimulationData.Signal;
signal2.Name = 'vel2_task2';
signal2.Values = timeseries(ww2*(1/rid), tt);
signal2.Values = setinterpmethod(signal2.Values,'linear'); 
ds2_task2 = Simulink.SimulationData.Dataset;
ds2_task2 = addElement(ds2_task2, signal2, 'velocity');
save('vel2_task2.mat', 'ds2_task2');

% Joint 3
ww3=qd_all(3,:);
signal3 = Simulink.SimulationData.Signal;
signal3.Name = 'vel3_task2';
signal3.Values = timeseries(ww3*(1/rid), tt);
signal3.Values = setinterpmethod(signal3.Values,'linear'); 
ds3_task2 = Simulink.SimulationData.Dataset;
ds3_task2 = addElement(ds3_task2, signal3, 'velocity');
save('vel3_task2.mat', 'ds3_task2');

% Joint 4
ww4=qd_all(4,:);
signal4 = Simulink.SimulationData.Signal;
signal4.Name = 'vel4_task2';
signal4.Values = timeseries(ww4*(1/rid), tt);
signal4.Values = setinterpmethod(signal4.Values,'linear'); 
ds4_task2 = Simulink.SimulationData.Dataset;
ds4_task2 = addElement(ds4_task2, signal4, 'velocity');
save('vel4_task2.mat', 'ds4_task2');

% START SIMULATION
sim('Simscape_task2_motors.slx');

% To-Workspace
figure;
sgtitle('Simscape Motor Torques');
subplot(2,2,1); plot(MotorTorque1t2.Time, MotorTorque1t2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q1 motor torque'); ylim([-0.09 0.09]);
subplot(2,2,2); plot(MotorTorque2t2.Time, MotorTorque2t2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q2 motor torque');
subplot(2,2,3); plot(MotorTorque3t2.Time, MotorTorque3t2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q3 motor torque');
subplot(2,2,4); plot(MotorTorque4t2.Time, MotorTorque4t2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q4 motor torque');

figure; % Matlab/SIMSCAPE comparison
sgtitle('MATLAB-Simscape Motor Torques comparison');
subplot(2,2,1); plot(MotorTorque1t2.Time, MotorTorque1t2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q1 motor torque'); ylim([-0.09 0.09]); hold on;
subplot(2,2,2); plot(MotorTorque2t2.Time, MotorTorque2t2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q2 motor torque'); hold on;
subplot(2,2,3); plot(MotorTorque3t2.Time, MotorTorque3t2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q3 motor torque'); hold on;
subplot(2,2,4); plot(MotorTorque4t2.Time, MotorTorque4t2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q4 motor torque'); hold on;
subplot(2,2,1); plot(t_all,q1tm_save,'color','b'); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q1 motor torque');
subplot(2,2,2); plot(t_all,q2tm_save,'color','b'); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q2 motor torque');
subplot(2,2,3); plot(t_all,q3tm_save,'color','b'); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q3 motor torque');
subplot(2,2,4); plot(t_all,q4tm_save,'color','b'); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q4 motor torque');

