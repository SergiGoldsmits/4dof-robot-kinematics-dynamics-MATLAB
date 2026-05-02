function PlotRobot3(Q,L,l0,col,fig,ll)
q1=Q(1); q2=Q(2); q3=Q(3); q4=Q(4);
l0=L(1); l1=L(2); l2=L(3); l3=L(4);
x0=0;
y0=0;
z0=l0;

%position of the end of the first link
R1 = l1 * cos(q2);
x1=R1*cos(q1);
y1=R1*sin(q1);
z1=z0+l1*sin(q2);

%position of the end of the second link
R2 = R1 + l2 * cos(q2+q3);
x2 = cos(q1) * R2;
y2 = sin(q1) * R2;
z2=z1+l2*sin(q2+q3);

%position of the end effector
R3 = R2 + l3 * cos(q2+q3+q4);
x3 = cos(q1) * R3;
y3 = sin(q1) * R3;
z3=z2+l3*sin(q2+q3+q4);

figure(fig); grid on;
set(ll,'XData', [0 x0*1e3 x1*1e3 x2*1e3 x3*1e3],...
    'YData', [0 z0*1e3 z1*1e3 z2*1e3 z3*1e3], 'color', col);
xlabel('X'); zlabel('Z');
axis equal;

app=(l1+l2+l3)*1.1*1e3;
xlim([-app app]);
ylim([-50 app]);


drawnow;
end