T21 = -G21 / G22;
T12 = -G12 / G11;
disp('T21 = ');
disp(T21);

disp('T12 = ');
disp(T12);

G11_p = G11 + T12;
G12_p = G12 + T21;
G21_p = G21 + T12;
G22_p = G22 + T21;

G_p = [G11_p G12_p; G21_p G22_p];

C11_p = pidtune(G11_p, 'PID');  
C22_p = pidtune(G22_p, 'PID'); 

C12_p = 0;  
C21_p = 0;  

C_p = [C11_p C12_p; C21_p C22_p];
disp(C_p);
T_p = feedback(C_p*G_p, eye(2)); 
figure;
step(T_p);

title('Step Response of 2x2 MIMO System with PID Controllers');