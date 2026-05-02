clearvars
close all
clc

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
q2_min = deg2rad(-120);    q2_max = deg2rad(120);
q3_min = deg2rad(-180);    q3_max = deg2rad(180);
q4_min = deg2rad(-180);    q4_max = deg2rad(180);

joint_limits = [q1_min q1_max;
                q2_min q2_max;
                q3_min q3_max;
                q4_min q4_max];

% Center of mass lengths [m]
g1=l1/2; 
g2=l2/2; 
g3=l3/2; 

% masses [kg]
m0 = 17;
m1 = 10;
m2 = 9;
m3 = 3;
m_payload=0.5; 

% Inertias [kg*m^2]
I0_xy = (1/4)*m0*r0^2 + (1/12)*m0*l0^2;
I0_z = (1/2)*m0*r0^2;
I1_yz = (1/4)*m1*r123^2 + (1/12)*m1*l1^2; 
I1_x = (1/2)*m1*r123^2;
I2_yz = (1/4)*m2*r123^2 + (1/12)*m2*l2^2; 
I2_x = (1/2)*m2*r123^2;
I3_yz = (1/4)*m3*r123^2 + (1/12)*m3*l3^2; 
I3_x = (1/2)*m3*r123^2;

% Mass Matrix
M=diag([m_payload, m_payload,m_payload, m3, m3, m3, I3_yz,...
    m2, m2, m2, I2_yz, m1, m1, m1, I1_yz, m0, m0, m0, ...
    I0_z]);  

% External forces
Fse_base = zeros(19,1);    

Fse_base(2)  = -m_payload*9.81; % Fz payload
Fse_base(5)  = -m3*9.81;        % Fz m3
Fse_base(9)  = -m2*9.81;        % Fz m2
Fse_base(13) = -m1*9.81;        % Fz m1
Fse_base(17) = -m0*9.81;        % Fz m0

% Helmet Visor reaction Force
F_helmet   = 4;        % [N]


%% --- Lines&Parabolas Motion Planning ---

% Waypoints is S-space
X    = [250 270 300 340 370 420 500 420 370 340 300 250 ] * 1e-3;
Z    = [ 75 75 75 150 180 220 230 220 180 150 75 75 ] * 1e-3;
Teta = deg2rad([0 5 5 -10 -15 -25 -35 -25 -15 -10 5 0]);
Y    = zeros(size(X));

L=[l0;l1;l2;l3;g1;g2;g3]; %[m]   %lenghts vector

nP = numel(X);           
S_points = [X;
            Y;
            Z ;
            Teta];

% S-coordinates of helmet approaching and tolerances
X_helmet = X(3);    % [m] 
Z_helmet = Z(3);    % [m] 
dx = 0.01; % [m] 
dz = 0.01; % [m] 

%%  Inverse Kinematics 

nJ = 4;   % number of joints               
Q  = zeros(nJ, nP);      

for i = 1:nP
    Q(:,i) = ROBOTinv_4R(S_points(:,i), L, -1);  
end

disp('Q (joint values per waypoint) [deg]:');
disp(rad2deg(Q));

% --- Joint limits check on waypoints ---

for i = 1:nP
    if any(Q(:,i) < joint_limits(:,1) | Q(:,i) > joint_limits(:,2))
        error("ERROR: waypoint %d NOT reachable within joint limits.", i);
    end
end

q1 = Q(1,:);
q2 = Q(2,:);
q3 = Q(3,:);
q4 = Q(4,:);

Q = [q1; q2; q3; q4];

dq1=diff(q1); 
dq2=diff(q2);
dq3=diff(q3);
dq4=diff(q4);


dQ=[dq1;dq2;dq3;dq4];
dQ_abs= abs(dQ);

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

%% Simulation sampling time
dt = 0.001;


%% 0) Calculate minimum actuation time
rise_time=zeros(4,size(dQ,2));
for i=1:size(dQ,2)
    rise_time(1,i)=min_rise_time(motor,dQ_abs(1,i));
    rise_time(2,i)=min_rise_time(motor,dQ_abs(2,i));
    rise_time(3,i)=min_rise_time(motor,dQ_abs(3,i));
    rise_time(4,i)=min_rise_time(motor,dQ_abs(4,i));

end

T_min_act= max(rise_time);

%% 1) Auxiliary point's transition time
t1= T_min_act(1)-sqrt(T_min_act(1)^2-2*dQ_abs(:,1)/motor.A);
tend=T_min_act(end)-sqrt(T_min_act(end)^2-2*dQ_abs(:,end)/motor.D);

%% 2) Internal parabolic transition tj;
min_act_time_joints = [T_min_act;T_min_act;T_min_act;T_min_act];
min_act_time_joints(:,1)=T_min_act(1)-0.5*t1;
min_act_time_joints(:,end)=T_min_act(end)-0.5*tend;

q_dot=dQ./min_act_time_joints; 

t_j = abs(diff(q_dot,[],2) / motor.A);   

t_j=[t1,t_j,tend];

%% 3) Time_vector
dt_j = zeros(4, size(min_act_time_joints,2));
for i=1:size(min_act_time_joints,2)
    dt_j(:,i)= min_act_time_joints(:,i)-0.5*(t_j(:,i)+t_j(:,i+1));
end

dt_j=[[0;0;0;0],dt_j];

t_vec= zeros(4,length(t_j)+length(dt_j));
for i=1:length(t_j)
    t_vec(:,i*2-1)=dt_j(:,i);
    t_vec(:,i*2)=t_j(:,i);
end

t_vec=cumsum(t_vec,2);

%% 4) Line coefficients
t_line_vec= cumsum([t1/2,min_act_time_joints],2); 

al=q_dot;
bl= Q(:,2:end)-al.*t_line_vec(:,2:end);

%% 5) Parabolic coefficent
q_dot_ext= [[0;0;0;0], q_dot,[0; 0;0;0] ];
Q_end_parabola =[ al, al(:,end)].*t_vec(:, 2:2:end) +[bl,bl(:,end)];


ap = diff(q_dot_ext, [] , 2)./(2*t_j);
bp = [q_dot, [0;0;0;0]] -ap*2.*t_vec(:, 2:2:end);
cp = (Q_end_parabola -ap.*t_vec(:, 2:2:end).^2-bp.*t_vec(:, 2:2:end));

static_joints = all(dQ_abs == 0, 2);
if any(static_joints)
    al(static_joints,:) = 0;
    ap(static_joints,:) = 0;
    bp(static_joints,:) = 0;
    cp(static_joints,:) = Q(static_joints,1);
    t_j(static_joints,:) = 1; 
end

%% 6) Complete Trajectory for each Joint

nPar = size(ap,2);      % n.of parabolic branches

% -- Joint 1 --
t1      = [];
q1_traj = [];
qp1_traj= [];
qpp1_traj=[];

for k = 1:nPar
   
    t_start_p = t_vec(1, 2*k-1);
    t_end_p   = t_vec(1, 2*k);
    tp = t_start_p:dt:t_end_p;
    
   
    if ~isempty(t1) && ~isempty(tp)
        tp = tp(2:end);
    end
    
    q_p   = ap(1,k).*tp.^2 + bp(1,k).*tp + cp(1,k);
    qp_p  = 2*ap(1,k).*tp + bp(1,k);
    qpp_p = 2*ap(1,k)*ones(size(tp));
    
    t1       = [t1,       tp];
    q1_traj  = [q1_traj,  q_p];
    qp1_traj = [qp1_traj, qp_p];
    qpp1_traj=[qpp1_traj,qpp_p];
    
    % --- Line between parabolas k and k+1 ---
    if k < nPar
        t_start_l = t_vec(1, 2*k);
        t_end_l   = t_vec(1, 2*k+1);
        tl = t_start_l:dt:t_end_l;
        
        if ~isempty(tl)
            tl = tl(2:end);  
        end
        
        q_l   = al(1,k).*tl + bl(1,k);
        qp_l  = al(1,k)*ones(size(tl));
        qpp_l = zeros(size(tl));
        
        t1       = [t1,       tl];
        q1_traj  = [q1_traj,  q_l];
        qp1_traj = [qp1_traj, qp_l];
        qpp1_traj=[qpp1_traj,qpp_l];
    end
end

% -- Joint 2 --
t2      = [];
q2_traj = [];
qp2_traj= [];
qpp2_traj=[];

for k = 1:nPar
    
    t_start_p = t_vec(2, 2*k-1);
    t_end_p   = t_vec(2, 2*k);
    tp = t_start_p:dt:t_end_p;
    
    if ~isempty(t2) && ~isempty(tp)
        tp = tp(2:end);
    end
    
    q_p   = ap(2,k).*tp.^2 + bp(2,k).*tp + cp(2,k);
    qp_p  = 2*ap(2,k).*tp + bp(2,k);
    qpp_p = 2*ap(2,k)*ones(size(tp));
    
    t2       = [t2,       tp];
    q2_traj  = [q2_traj,  q_p];
    qp2_traj = [qp2_traj, qp_p];
    qpp2_traj=[qpp2_traj,qpp_p];
    
    if k < nPar
        t_start_l = t_vec(2, 2*k);
        t_end_l   = t_vec(2, 2*k+1);
        tl = t_start_l:dt:t_end_l;
        
        if ~isempty(tl)
            tl = tl(2:end);
        end
        
        q_l   = al(2,k).*tl + bl(2,k);
        qp_l  = al(2,k)*ones(size(tl));
        qpp_l = zeros(size(tl));
        
        t2       = [t2,       tl];
        q2_traj  = [q2_traj,  q_l];
        qp2_traj = [qp2_traj, qp_l];
        qpp2_traj=[qpp2_traj,qpp_l];
    end
end

% -- Joint 3 --
t3      = [];
q3_traj = [];
qp3_traj= [];
qpp3_traj=[];

for k = 1:nPar
    
    t_start_p = t_vec(3, 2*k-1);
    t_end_p   = t_vec(3, 2*k);
    tp = t_start_p:dt:t_end_p;
    
    if ~isempty(t3) && ~isempty(tp)
        tp = tp(2:end);
    end
    
    q_p   = ap(3,k).*tp.^2 + bp(3,k).*tp + cp(3,k);
    qp_p  = 2*ap(3,k).*tp + bp(3,k);
    qpp_p = 2*ap(3,k)*ones(size(tp));
    
    t3       = [t3,       tp];
    q3_traj  = [q3_traj,  q_p];
    qp3_traj = [qp3_traj, qp_p];
    qpp3_traj=[qpp3_traj,qpp_p];
    
    if k < nPar
        t_start_l = t_vec(3, 2*k);
        t_end_l   = t_vec(3, 2*k+1);
        tl = t_start_l:dt:t_end_l;
        
        if ~isempty(tl)
            tl = tl(2:end);
        end
        
        q_l   = al(3,k).*tl + bl(3,k);
        qp_l  = al(3,k)*ones(size(tl));
        qpp_l = zeros(size(tl));
        
        t3       = [t3,       tl];
        q3_traj  = [q3_traj,  q_l];
        qp3_traj = [qp3_traj, qp_l];
        qpp3_traj=[qpp3_traj,qpp_l];
    end
end

% -- Joint 4 --
t4      = [];
q4_traj = [];
qp4_traj= [];
qpp4_traj=[];

for k = 1:nPar
    
    t_start_p = t_vec(4, 2*k-1);
    t_end_p   = t_vec(4, 2*k);
    tp = t_start_p:dt:t_end_p;
    
    if ~isempty(t4) && ~isempty(tp)
        tp = tp(2:end);
    end
    
    q_p   = ap(4,k).*tp.^2 + bp(4,k).*tp + cp(4,k);
    qp_p  = 2*ap(4,k).*tp + bp(4,k);
    qpp_p = 2*ap(4,k)*ones(size(tp));
    
    t4       = [t4,       tp];
    q4_traj  = [q4_traj,  q_p];
    qp4_traj = [qp4_traj, qp_p];
    qpp4_traj=[qpp4_traj,qpp_p];
    
    if k < nPar
        t_start_l = t_vec(4, 2*k);
        t_end_l   = t_vec(4, 2*k+1);
        tl = t_start_l:dt:t_end_l;
        
        if ~isempty(tl)
            tl = tl(2:end);
        end
        
        q_l   = al(4,k).*tl + bl(4,k);
        qp_l  = al(4,k)*ones(size(tl));
        qpp_l = zeros(size(tl));
        
        t4       = [t4,       tl];
        q4_traj  = [q4_traj,  q_l];
        qp4_traj = [qp4_traj, qp_l];
        qpp4_traj=[qpp4_traj,qpp_l];
    end
end

%% 6b) Construction of a unique t and q vector

% joint 1 as reference
t_all = t1;
N = length(t_all);

% Matrices 4xN
q_all   = zeros(4, N);
qp_all  = zeros(4, N);
qpp_all = zeros(4, N);

% --- Joint 1 ---
for k = 1:N
    q_all(1,k)   = q1_traj(k);
    qp_all(1,k)  = qp1_traj(k);
    qpp_all(1,k) = qpp1_traj(k);
end

% --- Joint 2 ---
N2 = length(t2);
for k = 1:N
    if k <= N2
        q_all(2,k)   = q2_traj(k);
        qp_all(2,k)  = qp2_traj(k);
        qpp_all(2,k) = qpp2_traj(k);
    else
        q_all(2,k)   = q2_traj(N2);
        qp_all(2,k)  = qp2_traj(N2);
        qpp_all(2,k) = qpp2_traj(N2);
    end
end

% --- Joint 3 ---
N3 = length(t3);
for k = 1:N
    if k <= N3
        q_all(3,k)   = q3_traj(k);
        qp_all(3,k)  = qp3_traj(k);
        qpp_all(3,k) = qpp3_traj(k);
    else
        q_all(3,k)   = q3_traj(N3);
        qp_all(3,k)  = qp3_traj(N3);
        qpp_all(3,k) = qpp3_traj(N3);
    end
end

% --- Joint 4 ---
N4 = length(t4);
for k = 1:N
    if k <= N4
        q_all(4,k)   = q4_traj(k);
        qp_all(4,k)  = qp4_traj(k);
        qpp_all(4,k) = qpp4_traj(k);
    else
        q_all(4,k)   = q4_traj(N4);
        qp_all(4,k)  = qp4_traj(N4);
        qpp_all(4,k) = qpp4_traj(N4);
    end
end



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

%------------------------------------------------------------%


%% --- TORQUES ---
nt = numel(t_all);
q1t_save = zeros(1, nt);
q2t_save = zeros(1, nt);
q3t_save = zeros(1, nt);
q4t_save = zeros(1, nt);
qt_save = zeros(4,nt);

q1tm_save = zeros(1, nt);
q2tm_save = zeros(1, nt);
q3tm_save = zeros(1, nt);
q4tm_save = zeros(1, nt);

F_helmet_on = zeros(1, nt);
for k = 1:nt

    t_helmet = t_all(k);
    qk   = q_all(:,k);    % 4x1
    qdk  = qp_all(:,k);   % 4x1
    qddk = qpp_all(:,k);  % 4x1

    % Dynamic Jacobian
    J_din     = ROBOTjacdin_4R(qk, L);          
    J_dot_din = ROBOTjacPdin_4R(qk, qdk, L);    

    % Extended accelerations
    Sdpp = J_dot_din*qdk + J_din*qddk;          

    % Inertial forces
    Fsi = -M*Sdpp;                              

    % External forces
    Fse = Fse_base;

    % Check the EE position 
    Se = ROBOTdir_4R(qk, L);  
    Xee = Se(1);
    Zee = Se(3);

    % Apply the Helmet Visor reaction force 
    if abs(Xee - X_helmet) <= dx && abs(Zee - Z_helmet) <= dz...
            && qp_all(4,k)<0
     
        Fse(2) = Fse(2) - F_helmet;
        F_helmet_on(k) = -F_helmet;

    elseif abs(Xee - X_helmet) <= dx && abs(Zee - Z_helmet) <= dz...
            && qp_all(4,k)>0

         Fse(2) = Fse(2) + F_helmet;
         F_helmet_on(k) = F_helmet;
    end

    % Total Forces
    Fs = Fse + Fsi;                            

    % Joints Torques
    Fcq = -J_din.' * Fs;                        

    
    q1t_save(k) = Fcq(1);
    q2t_save(k) = Fcq(2);
    q3t_save(k) = Fcq(3);
    q4t_save(k) = Fcq(4);
    qt_save(:,k) = Fcq;

    app_1(k) = q1t_save(k) * qpp_all(1,k);
    app_2(k) = q2t_save(k) * qpp_all(2,k);
    app_3(k) = q3t_save(k) * qpp_all(3,k);
    app_4(k) = q4t_save(k) * qpp_all(4,k);
    app(:,k) = Fcq .* qpp_all(:,k);
    

    % Motor Torques
    q1tm_save(k) = q1t_save(k)*rid + mot_inertia*qpp_all(1,k)*(1/rid);
    q2tm_save(k) = q2t_save(k)*rid + mot_inertia*qpp_all(2,k)*(1/rid);
    q3tm_save(k) = q3t_save(k)*rid + mot_inertia*qpp_all(3,k)*(1/rid);
    q4tm_save(k) = q4t_save(k)*rid + mot_inertia*qpp_all(4,k)*(1/rid);
end

% Load Factor check
beta = zeros(1,nJ);      
rid_opt = zeros(1,nJ);   
rid_max = zeros(1,nJ);
rid_min = zeros(1,nJ);

for p=1:nJ

    wrq(p) = rms(qp_all(p,:));
    dwrq(p) = rms(qpp_all(p,:));
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


%% --- TORQUES PLOTS ---

figure; % Joints Torques plots
subplot(2,2,1); plot(t_all,q1t_save); grid on; xlabel('Time [s]'); ylabel('T\_joint [Nm]'); title('q1 torque');
subplot(2,2,2); plot(t_all,q2t_save); grid on; xlabel('Time [s]'); ylabel('T\_joint [Nm]'); title('q2 torque');
subplot(2,2,3); plot(t_all,q3t_save); grid on; xlabel('Time [s]'); ylabel('T\_joint [Nm]'); title('q3 torque');
subplot(2,2,4); plot(t_all,q4t_save); grid on; xlabel('Time [s]'); ylabel('T\_joint [Nm]'); title('q4 torque');

figure; % Motor Torques plots
grid on;
subplot(2,2,1); plot(t_all,q1tm_save); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q1 motor torque');
subplot(2,2,2); plot(t_all,q2tm_save); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q2 motor torque');
subplot(2,2,3); plot(t_all,q3tm_save); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q3 motor torque');
subplot(2,2,4); plot(t_all,q4tm_save); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q4 motor torque');

figure; % Helmet's Visor Reaction Force Logic
plot(t_all, F_helmet_on, 'LineWidth', 1);
grid on;
xlabel('Time [s]');
ylabel('F\_helmet [N]');
title('Visor reaction force');


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
    plot(t_all, qp_all(j,:), 'LineWidth', 1.2);
    xlabel('t [s]');
    ylabel('q\_dot [rad/s]');
    title(sprintf('Joint %d - Velocity', j));
end

figure; % Acc
sgtitle('Joint Accelerations');

for j = 1:4
    subplot(2,2,j); hold on; grid on;
    plot(t_all, qpp_all(j,:), 'LineWidth', 1.2);
    xlabel('t [s]');
    ylabel('q\_ddot [rad/s^2]');
    title(sprintf('Joint %d - Acceleration', j));
end


%% --- EE Trajectory in S-space ---

S_all = zeros(4, nt);
for k = 1:nt
    S_all(:,k) = ROBOTdir_4R(q_all(:,k), L);
end

figure;
plot(S_all(1,:), S_all(3,:), 'b-', 'LineWidth', 2); hold on;
plot(S_all(1,1),   S_all(3,1),   'go', 'MarkerFaceColor', 'g');  % start
plot(S_all(1,end), S_all(3,end), 'ro', 'MarkerFaceColor', 'r');  % end

for p = 1:nP
    Sp = ROBOTdir_4R(Q(:,p), L);
    plot(Sp(1), Sp(3), 'kx', 'MarkerSize', 10, 'LineWidth', 2);   % waypoints
end

grid on; axis equal; box on;
xlabel('X [m]');
ylabel('Z [m]');
title('End-Effector trajectory (XZ-plan)');

% %% --- Animation ---
% f1 = figure;                      
% f1.WindowStyle='normal';
% hold on; 
% grid on;
% xlabel('X [mm]');
% ylabel('Z [mm]');
% axis equal;
% 
% app = (l1 + l2 + l3)*1.1*1e3;
% xlim([-app app]);
% ylim([-app app]);
% 
% % Linea robot (già la usi)
% ll1 = line('XData', [0 0], ...
%            'YData', [0 0], ...
%            'linestyle', '-', 'linewidth',2, 'color','r', ...
%            'marker','o','markersize',6,'markerfacecolor','k');
% 
% %  Linea SCIA (vuota all'inizio)
% trail = line('XData', [], ...
%              'YData', [], ...
%              'Color', [0 0.3 1], ...
%              'LineWidth', 1.5);
% 
% % vettori per salvare la scia accumulata
% trail_X = [];
% trail_Z = [];
% 
% for k = 1:size(q_all,2)
% 
%     % --- Plot robot istantaneo ---
%     PlotRobot3(q_all(:,k), L, l0, 'r', f1, ll1);
% 
%     % --- Aggiorna SCIA ---
%     Se = ROBOTdir_4R(q_all(:,k), L);   % end-effector in [X;Y;Z;theta]
%     Xe = Se(1)*1e3;                    % in mm
%     Ze = Se(3)*1e3;
% 
%     trail_X(end+1) = Xe;
%     trail_Z(end+1) = Ze;
% 
%     set(trail, 'XData', trail_X, 'YData', trail_Z);
% 
%     drawnow;
% end


%% --Simscape utilities--

%Signal editor

%joint1
tt=t_all;
ww1=qp_all(1,:);
signal1 = Simulink.SimulationData.Signal;
signal1.Name = 'vel1_task1';
signal1.Values = timeseries(ww1*(1/rid), tt);
ds1_task1 = Simulink.SimulationData.Dataset;
ds1_task1 = addElement(ds1_task1, signal1, 'velocity');
save('vel1_task1.mat', 'ds1_task1');

%joint2
tt=t_all;
ww2=qp_all(2,:);
signal2 = Simulink.SimulationData.Signal;
signal2.Name = 'vel2_task1';
signal2.Values = timeseries(ww2*(1/rid), tt);
ds2_task1 = Simulink.SimulationData.Dataset;
ds2_task1 = addElement(ds2_task1, signal2, 'velocity');
save('vel2_task1.mat', 'ds2_task1');

%joint3
tt=t_all;
ww3=qp_all(3,:);
signal3 = Simulink.SimulationData.Signal;
signal3.Name = 'vel3_task1';
signal3.Values = timeseries(ww3*(1/rid), tt);
ds3_task1 = Simulink.SimulationData.Dataset;
ds3_task1 = addElement(ds3_task1, signal3, 'velocity');
save('vel3_task1.mat', 'ds3_task1');

%joint4
tt=t_all;
ww4=qp_all(4,:);
signal4 = Simulink.SimulationData.Signal;
signal4.Name = 'vel4_task1';
signal4.Values = timeseries(ww4*(1/rid), tt);
ds4_task1 = Simulink.SimulationData.Dataset;
ds4_task1 = addElement(ds4_task1, signal4, 'velocity');
save('vel4_task1.mat', 'ds4_task1');

% START SIMULATION
sim('Simscape_task1_motors.slx');

% To-Workspace
figure;
sgtitle('Simscape Motor Torques');
subplot(2,2,1); plot(MotorTorque1.Time, MotorTorque1.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q1 motor torque'); ylim([-0.09 0.09]);
subplot(2,2,2); plot(MotorTorque2.Time, MotorTorque2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q2 motor torque');
subplot(2,2,3); plot(MotorTorque3.Time, MotorTorque3.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q3 motor torque');
subplot(2,2,4); plot(MotorTorque4.Time, MotorTorque4.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q4 motor torque');

figure; % Matlab/SIMSCAPE comparison
sgtitle('MATLAB-Simscape Motor Torques comparison');
subplot(2,2,1); plot(MotorTorque1.Time, MotorTorque1.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q1 motor torque'); ylim([-0.09 0.09]); hold on;
subplot(2,2,2); plot(MotorTorque2.Time, MotorTorque2.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q2 motor torque'); hold on;
subplot(2,2,3); plot(MotorTorque3.Time, MotorTorque3.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q3 motor torque'); hold on;
subplot(2,2,4); plot(MotorTorque4.Time, MotorTorque4.Data,'color','r'); grid on; xlabel('Time [s]'); ylabel('T\_motor [Nm]'); title('q4 motor torque'); hold on;
subplot(2,2,1); plot(t_all,q1tm_save,'color','b'); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q1 motor torque');
subplot(2,2,2); plot(t_all,q2tm_save,'color','b'); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q2 motor torque');
subplot(2,2,3); plot(t_all,q3tm_save,'color','b'); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q3 motor torque');
subplot(2,2,4); plot(t_all,q4tm_save,'color','b'); xlabel('Time [s]'); grid on; ylabel('T\_motor [Nm]'); title('q4 motor torque');