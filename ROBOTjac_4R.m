function J = ROBOTjac_4R(Q,L)

% Links lengths
l0=L(1);
l1=L(2);
l2=L(3);
l3=L(4);
q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);

% Abbreviations
c1 = cos(q1); s1 = sin(q1);
Q23 = q2 + q3;
Q234 = q2 + q3 + q4;
c2 = cos(q2); s2 = sin(q2);
c23 = cos(Q23); s23 = sin(Q23);
c234 = cos(Q234); s234 = sin(Q234);

% Partial computations 

R = l1 * c2 + l2 * c23 + l3 * c234;
dR_dq2 = -l1 * s2 - l2 * s23 - l3 * s234;
dR_dq3 = -l2 * s23 - l3 * s234;
dR_dq4 = -l3 * s234;

dpz_dq2 = l1 * c2 + l2 * c23 + l3 * c234;
dpz_dq3 = l2 * c23 + l3 * c234;
dpz_dq4 = l3 * c234;

% Jacobian matrix construction
J = zeros(4, 4);

J(1, 1) = -s1 * R;     
J(2, 1) = c1 * R;       
J(3, 1) = 0;            
J(4, 1) = 0;            

J(1, 2) = c1 * dR_dq2;  
J(2, 2) = s1 * dR_dq2; 
J(3, 2) = dpz_dq2;      
J(4, 2) = 1;            

J(1, 3) = c1 * dR_dq3;  
J(2, 3) = s1 * dR_dq3;  
J(3, 3) = dpz_dq3;      
J(4, 3) = 1;            

J(1, 4) = c1 * dR_dq4;  
J(2, 4) = s1 * dR_dq4;  
J(3, 4) = dpz_dq4;      
J(4, 4) = 1;                      

end