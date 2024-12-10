% DH (Denavit–Hartenberg) Table for PUMA560
%             theta  d      a      alpha  R/P    offset
L(1) =  Link([0      0      0      pi/2   0      0]);
L(2) =  Link([0      0      0.4318 0      0      0]);
L(3) =  Link([0      0.15   0.0203 -pi/2  0      0]);
L(4) =  Link([0      0.4318 0      pi/2   0      0]);
L(5) =  Link([0      0      0      -pi/2  0      0]);
L(6) =  Link([0      0      0      0      0      0]);

% Joint Angle Limits qlim
L(1).qlim = deg2rad([-160 160]);
L(2).qlim = deg2rad([-45 225]);
L(3).qlim = deg2rad([-225 45]);
L(4).qlim = deg2rad([-110 170]);
L(5).qlim = deg2rad([-100 100]);
L(6).qlim = deg2rad([-266 266]);

puma560 = SerialLink(L, 'name', 'PUMA560');

% disp(puma560);

% Different Configurations
qr = [0, pi/2, -pi/2, 0, 0, 0]; % ready pose, arm up
qz = [0, 0, 0, 0, 0, 0]; % zero angles, arm down
qn = [0, pi/4, pi, 0, pi/4, 0]; % nominal table top picking pose
qs = [0, 0, -pi/2, 0, 0, 0]; % straight and horizontal

qh = [0, 0, pi/2, 0, 0, 0]; % horizontal, but not straight

ru = deg2rad([0, 45, 180, 0, 45, 0]); % right-handed, elbow up configuration
rd = deg2rad([0, -47.8, 5.39, -180, 47.6, 180]); % right-handed, elbow down configuration
lu = deg2rad([152, -225, 5.39, 145, 55.8, 21.4]); % left-handed, elbow up configuration
ld = deg2rad([152, -132, 180, 38.6, 49.3, 152]); % left-handed, elbow down configuration


% Plot for zero config
puma560.plot(qz, 'movie', 'PUMA560.gif');


% Forward Kinematics
q_test = qr;
f_kine = puma560.fkine(q_test);
disp("joint angles");
disp(q_test);
disp("forward kinematics");
disp(f_kine);
puma560.plot(q_test, 'movie', 'PUMA560 qr.gif');


% Trajectory plot from one config to other
disp("ru");
disp(ru);
disp("rd");
disp(rd);

qt = jtraj(ru, rd, 100);
% puma560.plot(qt, 'movie', 'PUMA560 rd to ru.gif');

% Gimbal Lock cases (wrist singularity, where q6 depends on q4)

% case 1: zero configuration qz
qz_singu = [0, 0, 0, pi/2, 0, 0]
qz_singu_2 = [0, 0, 0, 0, 0, pi/2]

qs_singu = [0, 0, -pi/2, pi/2, 0, 0] % case 2: straight and horizontal configuration qs
qh_singu = [0, 0, pi/2, pi/2, 0, 0] % case 3: horizontal, but not straight configuration qh

puma560.plot(jtraj(qz, qz_singu, 100), 'movie', 'PUMA560 Singularity qz.gif');
puma560.plot(jtraj(qz, qz_singu_2, 100), 'movie', 'PUMA560 Singularity qz 2.gif');

puma560.plot(jtraj(qs, qs_singu, 100), 'movie', 'PUMA560 Singularity qs.gif');
puma560.plot(jtraj(qh, qh_singu, 100), 'movie', 'PUMA560 Singularity qh.gif');

% Inverse Kinematics
q_fk = ru;
q_ik_exact = rd;

T = puma560.fkine(q_fk);

% Initial guess for joint angles, added some radians to not give exact guess to let it iterate and converge
q0 = q_ik_exact + 0.5;

q_ik = puma560.ikine(T, q0);


fkine_for_ikine = T;
fkine_of_ikine = puma560.fkine(q_ikine);

disp("fkine config q_fk (ru)");
disp(ru);
disp("exact solution q_ik_exact (rd)");
disp(rd);
disp("initial guess q0 (rd + 0.5)");
disp(q0);
disp("iterated solution q_ik* (rd*)");
disp(q_ik);
disp("transformation matrix of fkine config q_fk (ru)");
disp(fkine_for_ikine);
disp("transformation matrix of ikine solution q_ik* (rd*)");
disp(fkine_of_ikine);


qrt_ikine = jtraj(q_fk, q_ik, 100);
puma560.plot(qrt_ikine, 'movie', 'PUMA560 IK.gif');

% puma560.teach;

% Plotting Workspace (ChatGPT)

n = 5; % computation time gets massive very quickly as n is increased
q1 = linspace(L(1).qlim(1), L(1).qlim(2), n);
q2 = linspace(L(2).qlim(1), L(2).qlim(2), n);
q3 = linspace(L(3).qlim(1), L(3).qlim(2), n);
q4 = linspace(L(4).qlim(1), L(4).qlim(2), n);
q5 = linspace(L(5).qlim(1), L(5).qlim(2), n);
q6 = linspace(L(6).qlim(1), L(6).qlim(2), n);

workspace_points = [];

for theta1 = q1
    for theta2 = q2
        for theta3 = q3
            for theta4 = q4
                for theta5 = q5
                    for theta6 = q6
                        q = [theta1, theta2, theta3, theta4, theta5, theta6];
                        T = puma560.fkine(q);
                        pos = T.t;
                        workspace_points = [workspace_points; pos'];
                    end
                end
            end
        end
    end
end

% Plot workspace points
figure;
plot3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), '.');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
title('PUMA560 Workspace');
saveas(gcf, 'PUMA560 Workspace.png');