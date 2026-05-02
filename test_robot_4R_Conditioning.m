clear; close all; clc;

%% --- ROBOT PARAMETERS ---
l0 = 200e-3; 
l1 = 329e-3;
l2 = 311.50e-3;
l3 = 106e-3;

L  = [l0; l1; l2; l3];

%% --- JOINTS LIMITS ---
% q1_min = deg2rad(-180);    q1_max = deg2rad(180);
% q2_min = deg2rad(-120);    q2_max = deg2rad(120);
% q3_min = deg2rad(-140);    q3_max = deg2rad(140);
% q4_min = deg2rad(-180);    q4_max = deg2rad(180);
%% --- JOINTS LIMITS ---
q1_min = deg2rad(-180);    q1_max = deg2rad(180);
q2_min = deg2rad(-180);    q2_max = deg2rad(180);
q3_min = deg2rad(-180);    q3_max = deg2rad(180);
q4_min = deg2rad(-180);    q4_max = deg2rad(180);




%% 1) KINEMATICS TEST

S = [0.300; 0.100; 0.400; deg2rad(-10)];   % desired EE pose


% Workspace range check
max_reach = l1 + l2 + l3;
if sqrt(S(1)^2 + S(2)^2 + (S(3)-l0)^2) > max_reach
    error("ERROR: Target S is outside the maximum workspace radius.");
end

Q1 = ROBOTinv_4R(S, L,  1);   % elbow-up
Q2 = ROBOTinv_4R(S, L, -1);   % elbow-down

Q1_deg = rad2deg(Q1);
Q2_deg = rad2deg(Q2);

% Joint limit check 
joint_limits = [q1_min q1_max;
                q2_min q2_max;
                q3_min q3_max;
                q4_min q4_max];

if any(Q1 < joint_limits(:,1) | Q1 > joint_limits(:,2))
    error("ERROR: Desired pose S is NOT reachable within joint limits (solution 1).");
end
if any(Q2 < joint_limits(:,1) | Q2 > joint_limits(:,2))
    error("ERROR: Desired pose S is NOT reachable within joint limits (solution 2).");
end

% Direct Kinematics check
S1 = ROBOTdir_4R(Q1, L);
S2 = ROBOTdir_4R(Q2, L);

%% --- QUALITY INDEX  ---

% Jacobians
J1 = ROBOTjac_4R(Q1, L);
J2 = ROBOTjac_4R(Q2, L);

% Linear parts
Jv1 = J1(1:3,:);
Jv2 = J2(1:3,:);

% Singular values
s1 = svd(Jv1);
s2 = svd(Jv2);

% Conditioning index k^{-1}
k1 = min(s1)/max(s1);
k2 = min(s2)/max(s2);

fprintf("\n=== Quality index (conditioning) ===\n");
fprintf("Solution 1 (elbow-up):   k^{-1} = %.4f\n", k1);
fprintf("Solution 2 (elbow-down): k^{-1} = %.4f\n", k2);

%% --- PLOT ROBOT IN 3D SPACE ---
f1=figure;                      
f1.WindowStyle='normal';
hold on; 
grid on;
view(3); % 3D view
ll1=line('XData', [0 0 0], ...
        'YData', [0 0 0], ...
        'ZData', [0 0 0],...
        'linestyle', '-','linewidth',2,'color','b',...
        'marker','o','markersize',6,'markerfacecolor','k'); %1st sol

ll2=line('XData', [0 0 0], ...
        'YData', [0 0 0], ...
        'ZData', [0 0 0],...
        'linestyle', '-','linewidth',2,'color','b',...
        'marker','o','markersize',6,'markerfacecolor','k'); %2nd sol

PlotRobot2(Q1,L,l0,'r',f1,ll1); % draw the 1 sol.
PlotRobot2(Q2,L,l0,'b',f1,ll2); % draw the 2 sol.

%% 2) WORKSPACE AND CONDITIONING

% Sampling
Nq1 = 13;     Nq2 = 13; 
Nq3 = 9;      Nq4 = 9;

q1_vec = linspace(q1_min, q1_max, Nq1);
q2_vec = linspace(q2_min, q2_max, Nq2);
q3_vec = linspace(q3_min, q3_max, Nq3);
q4_vec = linspace(q4_min, q4_max, Nq4);

X = []; Y = []; Z = []; Kinv = [];

for q1 = q1_vec
for q2 = q2_vec
for q3 = q3_vec
for q4 = q4_vec

    Q = [q1; q2; q3; q4];

    S_fk = ROBOTdir_4R(Q, L);

    if S_fk(3) < 0
        continue;
    end

    J = ROBOTjac_4R(Q, L);
    Jv = J(1:3,:);

    s = svd(Jv);
    smax = max(s);
    smin = min(s);

    if smax < 1e-8
        continue;
    end

    k_inv = smin/smax;

    X(end+1) = S_fk(1);
    Y(end+1) = S_fk(2);
    Z(end+1) = S_fk(3);
    Kinv(end+1) = k_inv;

end
end
end
end

% legend
ncol = 256;
cmap = [linspace(1,0,ncol)' zeros(ncol,1) linspace(0,1,ncol)'];

kmin = min(Kinv);
kmax = max(Kinv);

%% --- FIGURE: WORKSPACE 3D COLORED ---
figure;
scatter3(X, Y, Z, 15, Kinv, 'filled');
grid on; axis equal;
xlabel("X [m]"); ylabel("Y [m]"); zlabel("Z [m]");
title("Workspace by conditioning index k^{-1}");
colormap(cmap);
cb = colorbar;
cb.Label.String = "k^{-1}";


%% 3) WORKSPACE MAPPING

thr_primary = 0.2;
idx_primary = find(Kinv >= thr_primary);

Xp = X(idx_primary);
Yp = Y(idx_primary);
Zp = Z(idx_primary);

K_all     = convhull(X,  Y,  Z);
K_primary = convhull(Xp, Yp, Zp);


%% --- PLOTS ---
figure; hold on; grid on; axis equal; view(135,30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace");

% Secondary workspace
trisurf(K_all, X, Y, Z, 'FaceColor',[0.7 0.7 0.7], ...
        'FaceAlpha',0.3, 'EdgeColor','none');

% Primary workspace
trisurf(K_primary, Xp, Yp, Zp, 'FaceColor',[0 0 1], ...
        'FaceAlpha',0.4, 'EdgeColor','none');


plot3(0,0,0,'ko','MarkerFaceColor','k');

legend("Secondary workspace", ...
       "Primary workspace");
       
%% 3) CATALOG PLOT 

thr_primary = 0.2;

idx = (Z >= 0);
X = X(idx); Y = Y(idx); Z = Z(idx); Kinv = Kinv(idx);

fig = figure;
PlotArea4R_Catalog(L, fig, X, Y, Z, Kinv, thr_primary);
