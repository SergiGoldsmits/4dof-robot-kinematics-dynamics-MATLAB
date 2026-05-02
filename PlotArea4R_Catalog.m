function PlotArea4R_Catalog(L, fig, X, Y, Z, Kinv, thr_primary)

figure(fig); clf; hold on; grid on; axis equal;

l0 = L(1); l1 = L(2); l2 = L(3); l3 = L(4);

% --- safety filter (even if already filtered in main) ---
idx = (Z >= 0);
X = X(idx); Y = Y(idx); Z = Z(idx); Kinv = Kinv(idx);

% --- distance from shoulder (0,0,l0) ---
r_xy = hypot(X, Y);              % sqrt(X^2 + Y^2)
rho  = hypot(r_xy, (Z - l0));    % sqrt(X^2 + Y^2 + (Z-l0)^2)

% --- Secondary radius (theoretical max reach) ---
Rout = l1 + l2 + l3;

% --- Primary radius from sampled data (well-conditioned) ---
idxp = (Kinv >= thr_primary);
Rpri = max(rho(idxp));
if isempty(Rpri)
    Rpri = NaN;   % no primary points
end

% --- draw circles in X-Z section centered at shoulder (0,l0) ---
cx = 0; cz = l0;
fi = linspace(0, 2*pi, 400);

% Secondary (black)
plot(cx + Rout*cos(fi), cz + Rout*sin(fi), 'k-', 'LineWidth', 2);

% Primary (blue) - only if exists
if ~isnan(Rpri)
    plot(cx + Rpri*cos(fi), cz + Rpri*sin(fi), 'b-', 'LineWidth', 2);
end


% --- stylized robot stretched along +X at z=l0 ---
plot(0, 0, 'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);      % base
plot(0, l0, 'ks', 'MarkerFaceColor',[1 1 0], 'MarkerSize', 7); % shoulder
plot([0 0], [0 l0], 'k-', 'LineWidth', 3);                     % column

p1 = [0; l0];
p2 = [l1; l0];
p3 = [l1+l2; l0];
p4 = [l1+l2+l3; l0];

plot([p1(1) p2(1)], [p1(2) p2(2)], 'k-', 'LineWidth', 6);
plot([p2(1) p3(1)], [p2(2) p3(2)], 'k-', 'LineWidth', 6);
plot([p3(1) p4(1)], [p3(2) p4(2)], 'k-', 'LineWidth', 6);

plot(p2(1), p2(2), 'ks', 'MarkerFaceColor',[0.8 0.8 0.8], 'MarkerSize', 10);
plot(p3(1), p3(2), 'ks', 'MarkerFaceColor',[0.8 0.8 0.8], 'MarkerSize', 10);
plot(p4(1), p4(2), 'kd', 'MarkerFaceColor',[0.6 0.6 0.6], 'MarkerSize', 9);

% --- floor and axes ---
yline(0,'k-','LineWidth',1.2);
xlim([-1.05*Rout, 1.05*Rout]);
ylim([0, l0 + 1.05*Rout]);

xlabel('X [m]'); ylabel('Z [m]');
% legend (simple)
if ~isnan(Rpri)
    legend('Secondary (max reach)', 'Primary (well-conditioned)', ...
           'Location','northeast');
else
    legend('Secondary (max reach)','Location','northeast');
end
hold off
end
