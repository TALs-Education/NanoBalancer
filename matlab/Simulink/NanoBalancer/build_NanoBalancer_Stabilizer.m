% build_NanoBalancer_Stabilizer — (re)generates NanoBalancer_Stabilizer.slx
%
% Top level:   Bump -> [Balancer Plant] -> ZOH (BNO055 100 Hz) -> [LQR Controller] -> Saturation -> Plant
% Controller:  four branches selected by a Multiport Switch driven by SensorMode
%   1 ideal            u = -K*x(k)
%   2 1-delay          u = -K*x(k-1)
%   3 1-delay + pred   u = -K*(Ad*x(k-1) + Bd*u(k-1))
%   4 2-delay + pred   u = -K*(Ad*(Ad*x(k-2) + Bd*u(k-2)) + Bd*u(k-1))

here = fileparts(mfilename('fullpath'));
mdl  = 'NanoBalancer_Stabilizer';
if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']); if isfile(f), delete(f); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'PreLoadFcn', 'InitNanoBalancer', 'InitFcn', 'InitNanoBalancer', ...
    'StopTime', '4', 'Solver', 'ode45', 'MaxStep', '1e-3', 'RelTol', '1e-6');

lib.const = 'simulink/Sources/Constant';   lib.step = 'simulink/Sources/Step';
lib.gain  = 'simulink/Math Operations/Gain'; lib.sum = 'simulink/Math Operations/Sum';
lib.delay = 'simulink/Discrete/Delay';     lib.zoh  = 'simulink/Discrete/Zero-Order Hold';
lib.mps   = 'simulink/Signal Routing/Multiport Switch';
lib.sat   = 'simulink/Discontinuities/Saturation';
lib.mfun  = 'simulink/User-Defined Functions/MATLAB Function';
lib.integ = 'simulink/Continuous/Integrator';
lib.scope = 'simulink/Sinks/Scope';        lib.tows = 'simulink/Sinks/To Workspace';
lib.demux = 'simulink/Signal Routing/Demux'; lib.mux = 'simulink/Signal Routing/Mux';
lib.sub   = 'simulink/Ports & Subsystems/Subsystem';
lib.in    = 'simulink/Sources/In1';        lib.out = 'simulink/Sinks/Out1';

B = @(sys, name, src, pos, varargin) add_block(src, [sys '/' name], 'Position', pos, varargin{:});
L = @(sys, a, b) add_line(sys, a, b, 'autorouting', 'smart');

%% ---------------- Bump ----------------
B(mdl, 'Bump start', lib.step, [30 40 60 70], 'Time', 'bump_t', 'After', 'bump_F', 'SampleTime', '0');
B(mdl, 'Bump end',   lib.step, [30 100 60 130], 'Time', 'bump_t+bump_dt', 'After', 'bump_F', 'SampleTime', '0');
B(mdl, 'Bump force', lib.sum,  [100 70 120 100], 'Inputs', '+-', 'IconShape', 'round');
L(mdl, 'Bump start/1', 'Bump force/1'); L(mdl, 'Bump end/1', 'Bump force/2');

%% ---------------- Plant subsystem ----------------
P = [mdl '/Balancer Plant'];
B(mdl, 'Balancer Plant', lib.sub, [200 50 330 150]);
delete_line(P, 'In1/1', 'Out1/1'); delete_block([P '/In1']); delete_block([P '/Out1']);
B(P, 'u [V]',       lib.in,  [20 60 50 75]);
B(P, 'F bump [N]',  lib.in,  [20 120 50 135], 'Port', '2');
B(P, 'Nonlinear dynamics', lib.mfun, [150 40 330 170]);
B(P, 'States',      lib.integ, [400 90 440 120], 'InitialCondition', 'x0');
B(P, 'x',           lib.out, [520 98 550 112]);
B(P, 'I motor [A]', lib.out, [520 150 550 165], 'Port', '2');
B(P, 'tau [Nm]',    lib.out, [520 190 550 205], 'Port', '3');
code = strjoin({
'function [ds, I, tau] = fcn(s, u, F, NB_pp)'
'pp = NB_pp;'
'% Nonlinear wheeled inverted pendulum + DC motor with driver current limit.'
'% s = [theta; theta_dot; x; x_dot], u = voltage to both motors, F = force at CoM'
'm_b=pp(1); m_w=pp(2); l=pp(3); Ib=pp(4); Iw=pp(5); r=pp(6); g=pp(7);'
'Kt=pp(8); Ke=pp(9); R=pp(10); Ilim=pp(11); Inl=pp(12);'
'th = s(1); thd = s(2); xd = s(4);'
'wm = xd/r - thd;                         % motor output speed relative to body'
'I = (u - Ke*wm)/R;'
'I = min(max(I, -Ilim), Ilim);            % Nano Motor Carrier 0.5 A limit'
'Tm = Kt*I - Kt*Inl*tanh(wm/0.5);         % per motor, incl. no-load friction'
'tau = 2*Tm;'
'a = Ib + m_b*l^2;  b = m_b*r*l;  c = Iw + (m_b + m_w)*r^2;'
'M = [a, b*cos(th); b*cos(th), c];'
'f = [m_b*g*l*sin(th) - tau + F*l*cos(th); tau + b*sin(th)*thd^2 + F*r];'
'acc = M\f;'
'ds = [thd; acc(1); xd; r*acc(2)];'
}, newline);
rt = sfroot; ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [P '/Nonlinear dynamics']);
ch.Script = code;
pp = ch.find('-isa', 'Stateflow.Data', 'Name', 'NB_pp'); pp.Scope = 'Parameter'; pp.Props.Array.Size = '[1 12]';
s_ = ch.find('-isa', 'Stateflow.Data', 'Name', 's'); s_.Props.Array.Size = '[4 1]';
L(P, 'States/1', 'Nonlinear dynamics/1');
L(P, 'u [V]/1', 'Nonlinear dynamics/2');
L(P, 'F bump [N]/1', 'Nonlinear dynamics/3');
L(P, 'Nonlinear dynamics/1', 'States/1');
L(P, 'States/1', 'x/1');
L(P, 'Nonlinear dynamics/2', 'I motor [A]/1');
L(P, 'Nonlinear dynamics/3', 'tau [Nm]/1');

%% ---------------- BNO055 / encoder sampling ----------------
B(mdl, 'BNO055 + encoders 100 Hz', lib.zoh, [400 55 440 95], 'SampleTime', 'sampleTime');
L(mdl, 'Bump force/1', 'Balancer Plant/2');
L(mdl, 'Balancer Plant/1', 'BNO055 + encoders 100 Hz/1');

%% ---------------- Controller subsystem ----------------
C = [mdl '/LQR Controller'];
B(mdl, 'LQR Controller', lib.sub, [500 40 640 130]);
delete_line(C, 'In1/1', 'Out1/1'); delete_block([C '/In1']); delete_block([C '/Out1']);
B(C, 'x meas', lib.in, [20 200 50 215]);
B(C, 'u cmd',  lib.out, [1260 290 1290 305]);
dl = {'DelayLength', '1', 'InitialCondition', '0', 'SampleTime', 'sampleTime'};
dl2 = {'DelayLength', '2', 'InitialCondition', '0', 'SampleTime', 'sampleTime'};
B(C, 'x(k-1)', lib.delay, [150 260 190 290], dl{:});
B(C, 'x(k-2)', lib.delay, [150 380 190 410], dl2{:});
B(C, 'u(k-1)', lib.delay, [150 500 190 530], dl{:});
B(C, 'u(k-2)', lib.delay, [150 580 190 610], dl2{:});
mg = {'Multiplication', 'Matrix(K*u)'};
% mode 3: x_hat = Ad*x(k-1) + Bd*u(k-1)
B(C, 'Ad (3)', lib.gain, [300 260 360 290], 'Gain', 'NB.Ad', mg{:});
B(C, 'Bd (3)', lib.gain, [300 320 360 350], 'Gain', 'NB.Bd', mg{:});
B(C, 'x_hat (3)', lib.sum, [420 275 440 335], 'Inputs', '++');
% mode 4: x_hat = Ad*(Ad*x(k-2) + Bd*u(k-2)) + Bd*u(k-1)
B(C, 'Ad (4a)', lib.gain, [300 380 360 410], 'Gain', 'NB.Ad', mg{:});
B(C, 'Bd (4a)', lib.gain, [300 580 360 610], 'Gain', 'NB.Bd', mg{:});
B(C, 'x(k-1) pred', lib.sum, [420 400 440 460], 'Inputs', '++');
B(C, 'Ad (4b)', lib.gain, [500 415 560 445], 'Gain', 'NB.Ad', mg{:});
B(C, 'Bd (4b)', lib.gain, [500 500 560 530], 'Gain', 'NB.Bd', mg{:});
B(C, 'x_hat (4)', lib.sum, [620 430 640 490], 'Inputs', '++');
% selector -> state estimate
B(C, 'SensorMode', lib.const, [680 0 750 30], 'Value', 'SensorMode');
B(C, 'Sensor Mode switch', lib.mps, [800 20 840 500], 'Inputs', '4', 'DataPortOrder', 'One-based contiguous');
B(C, '-K', lib.gain, [900 245 960 275], 'Gain', '-NB.K', mg{:});
% friction feed-forward (gearmotor no-load friction, I_nl*R per motor)
B(C, 'Friction FF', 'simulink/User-Defined Functions/Fcn', [880 340 1000 370], ...
    'Expression', 'FrictionComp*NB.R*0.08*tanh((u(4)/NB.r - u(2))/0.5)');
B(C, 'u = u_lqr + u_ff', lib.sum, [1040 280 1060 320], 'Inputs', '++');
B(C, 'Motor voltage limit', lib.sat, [1110 285 1150 315], 'UpperLimit', 'NB.Vmax', 'LowerLimit', '-NB.Vmax');
B(C, 'u_lin = u - u_ff', lib.sum, [1110 420 1130 460], 'Inputs', '+-');
% wiring
L(C, 'x meas/1', 'x(k-1)/1');
L(C, 'x meas/1', 'x(k-2)/1');
L(C, 'x(k-1)/1', 'Ad (3)/1');
L(C, 'u(k-1)/1', 'Bd (3)/1');
L(C, 'Ad (3)/1', 'x_hat (3)/1'); L(C, 'Bd (3)/1', 'x_hat (3)/2');
L(C, 'x(k-2)/1', 'Ad (4a)/1');
L(C, 'u(k-2)/1', 'Bd (4a)/1');
L(C, 'Ad (4a)/1', 'x(k-1) pred/1'); L(C, 'Bd (4a)/1', 'x(k-1) pred/2');
L(C, 'x(k-1) pred/1', 'Ad (4b)/1');
L(C, 'u(k-1)/1', 'Bd (4b)/1');
L(C, 'Ad (4b)/1', 'x_hat (4)/1'); L(C, 'Bd (4b)/1', 'x_hat (4)/2');
L(C, 'SensorMode/1', 'Sensor Mode switch/1');
L(C, 'x meas/1',    'Sensor Mode switch/2');
L(C, 'x(k-1)/1',    'Sensor Mode switch/3');
L(C, 'x_hat (3)/1', 'Sensor Mode switch/4');
L(C, 'x_hat (4)/1', 'Sensor Mode switch/5');
L(C, 'Sensor Mode switch/1', '-K/1');
L(C, 'Sensor Mode switch/1', 'Friction FF/1');
L(C, '-K/1', 'u = u_lqr + u_ff/1');
L(C, 'Friction FF/1', 'u = u_lqr + u_ff/2');
L(C, 'u = u_lqr + u_ff/1', 'Motor voltage limit/1');
L(C, 'Motor voltage limit/1', 'u cmd/1');
L(C, 'Motor voltage limit/1', 'u_lin = u - u_ff/1');
L(C, 'Friction FF/1', 'u_lin = u - u_ff/2');
L(C, 'u_lin = u - u_ff/1', 'u(k-1)/1');
L(C, 'u_lin = u - u_ff/1', 'u(k-2)/1');

%% ---------------- Top-level controller wiring ----------------
% Modes 2-4 model the BNO055 latency inside the controller (x(k-1), x(k-2)),
% so the controller input is the sampled true state.
L(mdl, 'BNO055 + encoders 100 Hz/1', 'LQR Controller/1');
L(mdl, 'LQR Controller/1', 'Balancer Plant/1');

%% ---------------- Logging / scope ----------------
B(mdl, 'x demux', lib.demux, [470 180 475 260], 'Outputs', '4');
B(mdl, 'rad2deg', lib.gain, [520 180 560 200], 'Gain', '180/pi');
B(mdl, 'Scope mux', lib.mux, [620 180 625 280], 'Inputs', '4');
B(mdl, 'Scope', lib.scope, [680 210 720 250]);
L(mdl, 'Balancer Plant/1', 'x demux/1');
L(mdl, 'x demux/1', 'rad2deg/1');
L(mdl, 'rad2deg/1', 'Scope mux/1');
L(mdl, 'x demux/3', 'Scope mux/2');
L(mdl, 'LQR Controller/1', 'Scope mux/3');
L(mdl, 'Balancer Plant/2', 'Scope mux/4');
set_param([mdl '/Scope'], 'NumInputPorts', '1');
tw = {'SaveFormat', 'Structure With Time'};
B(mdl, 'log_x', lib.tows, [400 300 460 320], 'VariableName', 'log_x', tw{:});
B(mdl, 'log_u', lib.tows, [780 140 840 160], 'VariableName', 'log_u', tw{:});
B(mdl, 'log_I', lib.tows, [400 340 460 360], 'VariableName', 'log_I', tw{:});
B(mdl, 'log_tau', lib.tows, [400 380 460 400], 'VariableName', 'log_tau', tw{:});
L(mdl, 'Balancer Plant/1', 'log_x/1');
L(mdl, 'LQR Controller/1', 'log_u/1');
L(mdl, 'Balancer Plant/2', 'log_I/1');
L(mdl, 'Balancer Plant/3', 'log_tau/1');

% annotation
note = Simulink.Annotation([mdl '/NanoBalancer stabiliser']);
note.Text = sprintf(['NanoBalancer LQR stabiliser (100 Hz, BNO055)\n' ...
    'SensorMode (InitNanoBalancer.m / workspace): 1 ideal | 2 one-sample delay | ' ...
    '3 one-sample delay + predictor | 4 two-sample delay + predictor\n' ...
    'Bump: horizontal force bump_F [N] for bump_dt [s] at bump_t at the body CoM.\n' ...
    'Run run_NanoBalancer_Stabilizer.m to compare all modes.']);
note.Position = [30 420 700 480];

Simulink.BlockDiagram.arrangeSystem(C);
Simulink.BlockDiagram.arrangeSystem(P);
save_system(mdl, f);
fprintf('Built %s\n', f);
