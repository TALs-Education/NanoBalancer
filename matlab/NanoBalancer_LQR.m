%% NanoBalancer — model, LQR design and nonlinear simulation
% Reads physical parameters from NanoBalancer_Stabilization_Parameters.xlsx
% (sheet "Inputs", column B = key, column C = value). Falls back to built-in
% defaults if the workbook is missing. No toolboxes required.
%
% States  x = [theta; theta_dot; x; x_dot]
%   theta  body tilt from vertical [rad], + forward
%   x      wheel travel r*phi [m]
% Input   u = motor voltage command (same for both motors) [V], |u| <= 12 V
%
% Coordinate conventions match the Fusion model: wheel axle on world X,
% body CoM l above the axle.

clear; clc; close all;

%% ---------------- Parameters ----------------
here = fileparts(mfilename('fullpath'));
xlsx = fullfile(here, '..', 'NanoBalancer_Stabilization_Parameters.xlsx');
P = loadParams(xlsx);

% Wheel side
m_w   = 2*P.m_w1;                        % both wheels [kg]
Iw    = 2*P.Iw_spin1 + 2*P.J_rotor*P.N^2; % wheels + reflected rotors [kg m^2]
% Body (everything except wheels)
m_b   = P.M_tot - m_w;
l     = P.M_tot*P.y_com_tot/m_b;         % CoM height above axle [m]
Ib_ax = P.Ixx_tot - 2*P.Iw_spin1;        % body pitch inertia about axle
Ib    = Ib_ax - m_b*l^2;                 % about body CoM
r     = P.r_w;  g = P.g;

% Motor (per motor, at output shaft, linear model)
T_stall = P.T_stall_kgcm*0.0980665;
w_nl    = P.rpm_nl*2*pi/60;
R_m     = P.V_rated/P.I_stall;
Kt      = T_stall/(P.I_stall - P.I_nl);
Ke      = (P.V_rated - P.I_nl*R_m)/w_nl;
T_lim   = Kt*(P.I_lim - P.I_nl);         % per motor at 500 mA

fprintf('--- NanoBalancer parameters ---\n');
fprintf('m_b = %.1f g, l = %.1f mm, Ib = %.0f g cm^2, Iw = %.1f g cm^2\n', ...
        m_b*1e3, l*1e3, Ib*1e7, Iw*1e7);
fprintf('Kt = %.4f N m/A, Ke = %.4f V s/rad, R = %.1f ohm, T_lim/motor = %.1f mN m\n', ...
        Kt, Ke, R_m, T_lim*1e3);

%% ---------------- Linear model ----------------
a = Ib + m_b*l^2;  b = m_b*r*l;  c = Iw + (m_b + m_w)*r^2;  D = a*c - b^2;
mgl = m_b*g*l;
% torque input (tau = total torque of both motors on wheels)
A21 = c*mgl/D;  A41 = -r*b*mgl/D;  B2 = -(c + b)/D;  B4 = r*(a + b)/D;
% voltage input (both motors): tau = k1*u - k2*(phi_dot - theta_dot)
k1 = 2*Kt/R_m;  k2 = 2*Kt*Ke/R_m;
A = [0      1        0   0;
     A21    B2*k2    0  -B2*k2/r;
     0      0        0   1;
     A41    B4*k2    0  -B4*k2/r];
B = [0; B2*k1; 0; B4*k1];

p_ol = eig(A);
fprintf('\nOpen-loop poles: %s\n', mat2str(p_ol.', 4));
fprintf('Unstable pole: %.2f rad/s  (fall time constant %.0f ms)\n', max(real(p_ol)), 1e3/max(real(p_ol)));

%% ---------------- Discrete LQR ----------------
Ts = 1/100;                         % control loop period [s] (BNO055 fusion output = 100 Hz)
delay_steps = 1;                    % BNO055 sample latency, in control periods (0 = ideal)
[Ad, Bd] = c2d_zoh(A, B, Ts);

% Bryson's rule: 1/max_acceptable^2
th_max  = deg2rad(2);    % tilt
thd_max = deg2rad(30);   % tilt rate
x_max   = 0.10;          % position drift
xd_max  = 0.30;          % speed
u_max   = 6;             % volts (leave headroom below 12 V)
Q  = diag([1/th_max^2, 1/thd_max^2, 1/x_max^2, 1/xd_max^2]);
Rw = 1/u_max^2;

K = dlqr_iter(Ad, Bd, Q, Rw);       % u = -K*x
p_cl = eig(Ad - Bd*K);
fprintf('\nLQR @ %.0f Hz  K = [%.3f  %.3f  %.3f  %.3f]\n', 1/Ts, K);
fprintf('   (V/rad, V/(rad/s), V/m, V/(m/s))\n');
fprintf('   per degree: %.3f V/deg, %.4f V/(deg/s)\n', K(1)*pi/180, K(2)*pi/180);
fprintf('Closed-loop poles (continuous equiv.): %s\n', mat2str((log(p_cl)/Ts).', 3));

%% ---------------- Nonlinear simulation ----------------
sim.Tend   = 3.0;          % [s]
sim.dt     = 1e-4;         % integration step [s]
sim.x0     = [deg2rad(5); 0; 0; 0];   % initial 5 deg tilt
sim.push_t = 1.5;          % disturbance time [s]
sim.push_J = 0.03;         % horizontal impulse at body CoM [N s]
S = struct('m_b',m_b,'m_w',m_w,'l',l,'Ib',Ib,'Iw',Iw,'r',r,'g',g, ...
           'Kt',Kt,'Ke',Ke,'R',R_m,'Ilim',P.I_lim,'Inl',P.I_nl,'Vmax',P.V_drv);
ctl = struct('K',K,'Ts',Ts,'delay',delay_steps,'pred',true,'Ad',Ad,'Bd',Bd);
out = simulate(S, ctl, sim);

figure('Name','NanoBalancer LQR','Color','w','Position',[100 100 1000 700]);
subplot(3,2,1); plot(out.t, rad2deg(out.x(1,:))); grid on; ylabel('\theta [deg]'); title('Body tilt');
subplot(3,2,2); plot(out.t, out.x(3,:)*1e3); grid on; ylabel('x [mm]'); title('Wheel travel');
subplot(3,2,3); plot(out.t, out.u); grid on; ylabel('u [V]'); title('Voltage command (sat. \pm12 V)');
subplot(3,2,4); plot(out.t, out.I); grid on; ylabel('I [A]'); title('Motor current per motor (lim. 0.5 A)');
subplot(3,2,5); plot(out.t, out.tau*1e3); grid on; ylabel('\tau [mN m]'); xlabel('t [s]'); title('Total wheel torque');
subplot(3,2,6); plot(out.t, out.x(4,:)); grid on; ylabel('v [m/s]'); xlabel('t [s]'); title('Ground speed');
saveas(gcf, fullfile(here, 'NanoBalancer_LQR_sim.png'));

%% ---------------- Robustness: largest recoverable initial tilt ----------------
cases = { 'ideal sensor (no delay)',            0, false;
          '1-sample BNO055 delay, no predictor', 1, false;
          '1-sample BNO055 delay + predictor',   1, true;
          '2-sample delay + predictor',          2, true };
fprintf('\nLargest recoverable initial tilt @ %.0f Hz (nonlinear, 12 V / 0.5 A limits):\n', 1/Ts);
for i = 1:size(cases,1)
    c = ctl; c.delay = cases{i,2}; c.pred = cases{i,3};
    fprintf('  %-38s %5.1f deg\n', cases{i,1}, maxTilt(S, c, sim));
end

%% ---------------- Export gains for the firmware ----------------
fid = fopen(fullfile(here, 'lqr_gains.h'), 'w');
fprintf(fid, '// Auto-generated by NanoBalancer_LQR.m  (%s)\n', char(datetime('now')));
fprintf(fid, '// u[V] = -(K0*theta[rad] + K1*theta_dot[rad/s] + K2*x[m] + K3*x_dot[m/s])\n');
fprintf(fid, '// Loop rate %.0f Hz. Voltage applies to both motors; PWM duty = u / 12 V.\n', 1/Ts);
fprintf(fid, '#pragma once\n');
fprintf(fid, 'const float LQR_TS = %.6ff;\n', Ts);
fprintf(fid, 'const float LQR_K[4] = {%.6ff, %.6ff, %.6ff, %.6ff};\n', K);
fprintf(fid, '// Delay predictor: x_hat = Ad*x_meas(k-1) + Bd*u(k-1)\n');
fprintf(fid, 'const float LQR_AD[4][4] = {{%.6ff,%.6ff,%.6ff,%.6ff},{%.6ff,%.6ff,%.6ff,%.6ff},{%.6ff,%.6ff,%.6ff,%.6ff},{%.6ff,%.6ff,%.6ff,%.6ff}};\n', Ad.');
fprintf(fid, 'const float LQR_BD[4] = {%.6ff, %.6ff, %.6ff, %.6ff};\n', Bd);
fprintf(fid, 'const float WHEEL_RADIUS_M = %.4ff;\n', r);
fprintf(fid, 'const float MOTOR_VMAX = %.2ff;\n', P.V_drv);
fclose(fid);
fprintf('Gains written to %s\n', fullfile(here, 'lqr_gains.h'));

% Workspace file for the Simulink controller model
NB = struct('Ts',Ts,'K',K,'Ad',Ad,'Bd',Bd,'A',A,'B',B,'delay_steps',delay_steps, ...
            'Vmax',P.V_drv,'Ilim',P.I_lim,'r',r,'Kt',Kt,'Ke',Ke,'R',R_m, ...
            'm_b',m_b,'m_w',m_w,'l',l,'Ib',Ib,'Iw',Iw,'g',g);
save(fullfile(here, 'NanoBalancer_params.mat'), 'NB');
fprintf('Simulink parameters saved to NanoBalancer_params.mat (struct NB)\n');

%% ======================= local functions =======================
function P = loadParams(xlsx)
    % defaults (values at CoM_TrayOffset = 8 mm, sliced masses)
    P = struct('g',9.81,'M_tot',0.42644,'y_com_tot',0.133237,'z_com_tot',-4.3e-6, ...
        'Ixx_tot',0.0107331,'Iyy_tot',3.833e-4,'Izz_tot',0.0108836, ...
        'm_w1',0.0032010,'Iw_spin1',8.166e-7,'r_w',0.020,'track',0.08427, ...
        'V_rated',12,'N',29.86,'rpm_nl',1100,'I_nl',0.08,'I_stall',0.75, ...
        'T_stall_kgcm',0.39,'J_rotor',3e-9,'V_drv',12,'I_lim',0.5);
    if isfile(xlsx)
        C = readcell(xlsx, 'Sheet', 'Inputs');
        n = 0;
        for i = 1:size(C,1)
            k = C{i,2}; v = C{i,3};
            if (ischar(k) || isstring(k)) && isnumeric(v) && isscalar(v) && ~ismissing(v)
                P.(char(k)) = v; n = n + 1;
            end
        end
        fprintf('Loaded %d parameters from %s\n', n, xlsx);
    else
        warning('Workbook not found, using built-in defaults.');
    end
end

function [Ad, Bd] = c2d_zoh(A, B, Ts)
    n = size(A,1); m = size(B,2);
    E = expm([A B; zeros(m, n+m)]*Ts);
    Ad = E(1:n,1:n); Bd = E(1:n,n+1:end);
end

function K = dlqr_iter(A, B, Q, R)
    Pm = Q;
    for i = 1:20000
        K  = (R + B'*Pm*B) \ (B'*Pm*A);
        Pn = Q + A'*Pm*(A - B*K);
        if norm(Pn - Pm, 'fro') < 1e-10*norm(Pm, 'fro'), Pm = Pn; break; end
        Pm = Pn;
    end
    K = (R + B'*Pm*B) \ (B'*Pm*A);
end

function deg = maxTilt(S, ctl, sim)
    lo = 0; hi = 60;
    for k = 1:14
        mid = (lo + hi)/2;
        s2 = sim; s2.x0 = [deg2rad(mid);0;0;0]; s2.push_J = 0; s2.Tend = 3;
        o2 = simulate(S, ctl, s2);
        if all(abs(o2.x(1,:)) < deg2rad(80)) && abs(o2.x(1,end)) < deg2rad(1)
            lo = mid;
        else
            hi = mid;
        end
    end
    deg = lo;
end

function out = simulate(S, ctl, sim)
    % Discrete controller at ctl.Ts. The measured state is ctl.delay samples
    % old (BNO055 latency). With ctl.pred, the controller propagates the old
    % measurement forward with the model and the inputs it already applied.
    N  = round(sim.Tend/sim.dt);
    nc = round(ctl.Ts/sim.dt);
    s  = sim.x0;                 % [theta; theta_dot; x; x_dot]
    out.t = (0:N)*sim.dt; out.x = zeros(4,N+1); out.u = zeros(1,N+1);
    out.I = zeros(1,N+1); out.tau = zeros(1,N+1);
    u = 0; pushed = false;
    d = ctl.delay;
    xbuf = repmat(s, 1, d+1);    % sampled states, newest last
    ubuf = zeros(1, max(d,1));   % applied inputs, newest last
    for k = 0:N
        if mod(k, nc) == 0
            xbuf = [xbuf(:,2:end), s];
            xm = xbuf(:,1);                       % measurement d samples old
            if ctl.pred && d > 0
                for j = 1:d, xm = ctl.Ad*xm + ctl.Bd*ubuf(end-d+j); end
            end
            u = -ctl.K*xm;  u = max(-S.Vmax, min(S.Vmax, u));   % ZOH
            ubuf = [ubuf(2:end), u];
        end
        if ~pushed && k*sim.dt >= sim.push_t && sim.push_J > 0
            % impulse at body CoM: horizontal momentum -> tilt rate
            s(2) = s(2) + sim.push_J*S.l/(S.Ib + S.m_b*S.l^2);
            pushed = true;
        end
        [ds, I, tau] = dyn(s, u, S);
        out.x(:,k+1) = s; out.u(k+1) = u; out.I(k+1) = I; out.tau(k+1) = tau;
        % RK4
        k1 = ds;
        k2 = dyn(s + 0.5*sim.dt*k1, u, S);
        k3 = dyn(s + 0.5*sim.dt*k2, u, S);
        k4 = dyn(s + sim.dt*k3, u, S);
        s  = s + sim.dt/6*(k1 + 2*k2 + 2*k3 + k4);
        if abs(s(1)) > pi/2, out.x(:,k+2:end) = repmat(s,1,N-k); break; end
    end
end

function [ds, I, tau] = dyn(s, u, S)
    th = s(1); thd = s(2); xd = s(4);
    phid = xd/S.r;
    wm = phid - thd;                         % motor output speed rel. to body
    I = (u - S.Ke*wm)/S.R;                   % per motor
    I = max(-S.Ilim, min(S.Ilim, I));        % driver current limit
    Tm = S.Kt*I - S.Kt*S.Inl*tanh(wm/0.5);   % no-load friction (smoothed sign)
    tau = 2*Tm;                              % both motors
    a = S.Ib + S.m_b*S.l^2;  bb = S.m_b*S.r*S.l;  c = S.Iw + (S.m_b + S.m_w)*S.r^2;
    M = [a, bb*cos(th); bb*cos(th), c];
    f = [S.m_b*S.g*S.l*sin(th) - tau; tau + bb*sin(th)*thd^2];
    acc = M \ f;                             % [theta_dd; phi_dd]
    ds = [thd; acc(1); xd; S.r*acc(2)];
end
