% build_NanoBalancer_Encoder — balancer with Simscape gearmotors + magnetic encoders
%
%  Bump -> [Body Dynamics] --w_rel--> [Gearmotor L], [Gearmotor R] --tau--> Body
%  Body theta/theta_dot  ---\
%  Encoder counts L/R ------> [Sensors 100 Hz] -> [LQR Controller] -> u -> both motors
%
% EncoderMode (workspace): 1 = true wheel state (ideal), 2 = from encoder counts
% SensorMode  (workspace): same 4 settings as NanoBalancer_Stabilizer.slx
here = fileparts(mfilename('fullpath')); cd(here);
mdl = 'NanoBalancer_Encoder';
if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']); if isfile(f), delete(f); end
InitNanoBalancer;
new_system(mdl); open_system(mdl);
set_param(mdl, 'PreLoadFcn', 'InitNanoBalancer', 'InitFcn', 'InitNanoBalancer', ...
    'StopTime', '4', 'Solver', 'daessc', 'MaxStep', '1e-4', 'RelTol', '1e-4');
B = @(sys, n, src, p, varargin) add_block(src, [sys '/' n], 'Position', p, varargin{:});
L = @(sys, a, b) add_line(sys, a, b, 'autorouting', 'smart');

%% bump
B(mdl, 'Bump start', 'simulink/Sources/Step', [30 40 60 70], 'Time', 'bump_t', 'After', 'bump_F', 'SampleTime', '0');
B(mdl, 'Bump end',   'simulink/Sources/Step', [30 100 60 130], 'Time', 'bump_t+bump_dt', 'After', 'bump_F', 'SampleTime', '0');
B(mdl, 'Bump force', 'simulink/Math Operations/Sum', [100 70 120 100], 'Inputs', '+-', 'IconShape', 'round');
L(mdl, 'Bump start/1', 'Bump force/1'); L(mdl, 'Bump end/1', 'Bump force/2');

%% body dynamics (motor torque comes from Simscape)
P = [mdl '/Body Dynamics'];
B(mdl, 'Body Dynamics', 'simulink/Ports & Subsystems/Subsystem', [200 50 330 150]);
delete_line(P, 'In1/1', 'Out1/1'); delete_block([P '/In1']); delete_block([P '/Out1']);
B(P, 'tau wheels [Nm]', 'simulink/Sources/In1', [20 60 50 75]);
B(P, 'F bump [N]', 'simulink/Sources/In1', [20 120 50 135], 'Port', '2');
B(P, 'Body + wheel dynamics', 'simulink/User-Defined Functions/MATLAB Function', [150 40 330 170]);
B(P, 'States', 'simulink/Continuous/Integrator', [400 90 440 120], 'InitialCondition', 'x0');
B(P, 'x', 'simulink/Sinks/Out1', [520 98 550 112]);
B(P, 'w_rel', 'simulink/Sinks/Out1', [520 150 550 165], 'Port', '2');
code = strjoin({
'function ds = fcn(s, tau, F, NB_pb)'
'% Wheeled inverted pendulum, body + wheels only (motor rotor inertia, friction'
'% and torque come from the Simscape gearmotors). tau = total torque on wheels.'
'm_b=NB_pb(1); m_w=NB_pb(2); l=NB_pb(3); Ib=NB_pb(4); Iw=NB_pb(5); r=NB_pb(6); g=NB_pb(7);'
'th = s(1); thd = s(2); xd = s(4);'
'a = Ib + m_b*l^2;  b = m_b*r*l;  c = Iw + (m_b + m_w)*r^2;'
'M = [a, b*cos(th); b*cos(th), c];'
'f = [m_b*g*l*sin(th) - tau + F*l*cos(th); tau + b*sin(th)*thd^2 + F*r];'
'acc = M\f;'
'ds = [thd; acc(1); xd; r*acc(2)];'
}, newline);
rt = sfroot; ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [P '/Body + wheel dynamics']); ch.Script = code;
d = ch.find('-isa', 'Stateflow.Data', 'Name', 'NB_pb'); d.Scope = 'Parameter'; d.Props.Array.Size = '[1 7]';
d = ch.find('-isa', 'Stateflow.Data', 'Name', 's'); d.Props.Array.Size = '[4 1]';
L(P, 'States/1', 'Body + wheel dynamics/1'); L(P, 'tau wheels [Nm]/1', 'Body + wheel dynamics/2');
L(P, 'F bump [N]/1', 'Body + wheel dynamics/3'); L(P, 'Body + wheel dynamics/1', 'States/1');
B(P, 'w_rel calc', 'simulink/Math Operations/Gain', [440 150 500 175], 'Gain', '[0 -1 0 1/NB.r]', 'Multiplication', 'Matrix(K*u)');
L(P, 'States/1', 'x/1'); L(P, 'States/1', 'w_rel calc/1'); L(P, 'w_rel calc/1', 'w_rel/1');

%% gearmotors
nb_add_gearmotor(mdl, 'Gearmotor L', [420 200 560 300]);
nb_add_gearmotor(mdl, 'Gearmotor R', [420 340 560 440]);
B(mdl, 'tau L+R', 'simulink/Math Operations/Sum', [620 280 640 300], 'Inputs', '++', 'IconShape', 'round');
L(mdl, 'Body Dynamics/2', 'Gearmotor L/2'); L(mdl, 'Body Dynamics/2', 'Gearmotor R/2');
L(mdl, 'Gearmotor L/1', 'tau L+R/1'); L(mdl, 'Gearmotor R/1', 'tau L+R/2');
L(mdl, 'tau L+R/1', 'Body Dynamics/1'); L(mdl, 'Bump force/1', 'Body Dynamics/2');

%% sensors @ 100 Hz
S = [mdl '/Sensors 100 Hz'];
B(mdl, 'Sensors 100 Hz', 'simulink/Ports & Subsystems/Subsystem', [700 40 840 160]);
delete_line(S, 'In1/1', 'Out1/1'); delete_block([S '/In1']); delete_block([S '/Out1']);
B(S, 'x true', 'simulink/Sources/In1', [20 40 50 55]);
B(S, 'count L', 'simulink/Sources/In1', [20 200 50 215], 'Port', '2');
B(S, 'count R', 'simulink/Sources/In1', [20 260 50 275], 'Port', '3');
B(S, 'x meas', 'simulink/Sinks/Out1', [900 140 930 155]);
zoh = {'SampleTime', 'sampleTime'};
B(S, 'ZOH x', 'simulink/Discrete/Zero-Order Hold', [100 35 140 60], zoh{:});
B(S, 'ZOH cnt L', 'simulink/Discrete/Zero-Order Hold', [100 195 140 220], zoh{:});
B(S, 'ZOH cnt R', 'simulink/Discrete/Zero-Order Hold', [100 255 140 280], zoh{:});
B(S, 'x demux', 'simulink/Signal Routing/Demux', [180 30 185 110], 'Outputs', '4');
B(S, 'mean count', 'simulink/Math Operations/Sum', [190 220 210 260], 'Inputs', '++');
B(S, 'counts -> psi [rad]', 'simulink/Math Operations/Gain', [240 225 300 255], 'Gain', 'GM.cnt2rad/2');
B(S, 'psi(k-1)', 'simulink/Discrete/Delay', [330 290 370 320], 'DelayLength', '1', 'InitialCondition', '0', zoh{:});
B(S, 'dpsi', 'simulink/Math Operations/Sum', [400 260 420 300], 'Inputs', '+-');
B(S, 'inv Ts', 'simulink/Math Operations/Gain', [450 265 490 295], 'Gain', '1/sampleTime');
B(S, 'psi + theta', 'simulink/Math Operations/Sum', [540 180 560 220], 'Inputs', '++');
B(S, 'speed LPF', 'simulink/Discrete/Discrete Transfer Fcn', [470 330 540 360], 'Numerator', '[enc_a]', 'Denominator', '[1 enc_a-1]', 'SampleTime', 'sampleTime');
B(S, 'psidot + thetadot', 'simulink/Math Operations/Sum', [540 250 560 290], 'Inputs', '++');
B(S, 'r (x)', 'simulink/Math Operations/Gain', [600 185 640 215], 'Gain', 'NB.r');
B(S, 'r (xdot)', 'simulink/Math Operations/Gain', [600 255 640 285], 'Gain', 'NB.r');
B(S, 'x from encoders', 'simulink/Signal Routing/Mux', [700 100 705 300], 'Inputs', '4');
B(S, 'EncoderMode', 'simulink/Sources/Constant', [700 20 760 45], 'Value', 'EncoderMode');
B(S, 'Encoder Mode switch', 'simulink/Signal Routing/Multiport Switch', [800 20 840 280], 'Inputs', '3', 'DataPortOrder', 'One-based contiguous');
% --- steady-state Kalman observer: y = [theta; theta_dot; x_enc], model + applied voltage
B(S, 'u cmd', 'simulink/Sources/In1', [20 420 50 435], 'Port', '4');
B(S, 'u(k-1)', 'simulink/Discrete/Delay', [100 415 140 440], 'DelayLength', '1', 'InitialCondition', '0', zoh{:});
B(S, 'y mux', 'simulink/Signal Routing/Mux', [660 360 665 440], 'Inputs', '3');
B(S, 'Kalman observer', 'simulink/User-Defined Functions/MATLAB Function', [700 360 800 440]);
kcode = strjoin({
'function xh = kf(y, u_prev, KF_Ad, KF_Bd, KF_L, KF_ff)'
'% Steady-state Kalman observer (current estimator), runs at the 100 Hz loop rate.'
'% y = [theta; theta_dot; x_enc]. Predict with the linear model and the linear'
'% part of the applied voltage (friction feed-forward removed), then correct.'
'persistent x'
'if isempty(x), x = zeros(4,1); end'
'u_lin = u_prev - KF_ff(1)*tanh((x(4)/KF_ff(2) - x(2))/0.5);'
'xp = KF_Ad*x + KF_Bd*u_lin;'
'C = [1 0 0 0; 0 1 0 0; 0 0 1 0];'
'x = xp + KF_L*(y - C*xp);'
'xh = x;'
}, newline);
rt2 = sfroot; kch = rt2.find('-isa', 'Stateflow.EMChart', 'Path', [S '/Kalman observer']); kch.Script = kcode;
for nm = {'KF_Ad','KF_Bd','KF_L','KF_ff'}, dd = kch.find('-isa','Stateflow.Data','Name',nm{1}); dd.Scope = 'Parameter'; end
dd = kch.find('-isa','Stateflow.Data','Name','y'); dd.Props.Array.Size = '[3 1]';
L(S, 'x true/1', 'ZOH x/1'); L(S, 'count L/1', 'ZOH cnt L/1'); L(S, 'count R/1', 'ZOH cnt R/1');
L(S, 'ZOH x/1', 'x demux/1');
L(S, 'ZOH cnt L/1', 'mean count/1'); L(S, 'ZOH cnt R/1', 'mean count/2');
L(S, 'mean count/1', 'counts -> psi [rad]/1');
L(S, 'counts -> psi [rad]/1', 'psi(k-1)/1');
L(S, 'counts -> psi [rad]/1', 'dpsi/1'); L(S, 'psi(k-1)/1', 'dpsi/2');
L(S, 'dpsi/1', 'inv Ts/1');
L(S, 'counts -> psi [rad]/1', 'psi + theta/1'); L(S, 'x demux/1', 'psi + theta/2');
L(S, 'inv Ts/1', 'speed LPF/1'); L(S, 'speed LPF/1', 'psidot + thetadot/1'); L(S, 'x demux/2', 'psidot + thetadot/2');
L(S, 'psi + theta/1', 'r (x)/1'); L(S, 'psidot + thetadot/1', 'r (xdot)/1');
L(S, 'x demux/1', 'x from encoders/1'); L(S, 'x demux/2', 'x from encoders/2');
L(S, 'r (x)/1', 'x from encoders/3'); L(S, 'r (xdot)/1', 'x from encoders/4');
L(S, 'EncoderMode/1', 'Encoder Mode switch/1');
L(S, 'ZOH x/1', 'Encoder Mode switch/2');
L(S, 'x from encoders/1', 'Encoder Mode switch/3');
L(S, 'u cmd/1', 'u(k-1)/1');
L(S, 'x demux/1', 'y mux/1'); L(S, 'x demux/2', 'y mux/2'); L(S, 'r (x)/1', 'y mux/3');
L(S, 'y mux/1', 'Kalman observer/1'); L(S, 'u(k-1)/1', 'Kalman observer/2');
L(S, 'Kalman observer/1', 'Encoder Mode switch/4');
L(S, 'Encoder Mode switch/1', 'x meas/1');
L(mdl, 'Body Dynamics/1', 'Sensors 100 Hz/1');
L(mdl, 'Gearmotor L/3', 'Sensors 100 Hz/2');
L(mdl, 'Gearmotor R/3', 'Sensors 100 Hz/3');

%% controller (copied from the validated stabiliser model)
src = 'NanoBalancer_Stabilizer';
if ~bdIsLoaded(src), load_system(fullfile(here, [src '.slx'])); end
add_block([src '/LQR Controller'], [mdl '/LQR Controller'], 'Position', [900 60 1040 140]);
L(mdl, 'Sensors 100 Hz/1', 'LQR Controller/1');
L(mdl, 'LQR Controller/1', 'Gearmotor L/1');
L(mdl, 'LQR Controller/1', 'Gearmotor R/1');
L(mdl, 'LQR Controller/1', 'Sensors 100 Hz/4');

%% logging
tw = {'SaveFormat', 'Structure With Time'};
B(mdl, 'log_x',  'simulink/Sinks/To Workspace', [420 480 480 500], 'VariableName', 'log_x', tw{:});
B(mdl, 'log_xm', 'simulink/Sinks/To Workspace', [900 200 960 220], 'VariableName', 'log_xm', tw{:});
B(mdl, 'log_u',  'simulink/Sinks/To Workspace', [1100 60 1160 80], 'VariableName', 'log_u', tw{:});
B(mdl, 'log_I',  'simulink/Sinks/To Workspace', [620 200 680 220], 'VariableName', 'log_I', tw{:});
B(mdl, 'log_cnt','simulink/Sinks/To Workspace', [620 400 680 420], 'VariableName', 'log_cnt', tw{:});
L(mdl, 'Body Dynamics/1', 'log_x/1'); L(mdl, 'Sensors 100 Hz/1', 'log_xm/1');
L(mdl, 'LQR Controller/1', 'log_u/1'); L(mdl, 'Gearmotor L/2', 'log_I/1'); L(mdl, 'Gearmotor L/3', 'log_cnt/1');
B(mdl, 'rad2deg', 'simulink/Math Operations/Gain', [1100 200 1140 220], 'Gain', '180/pi');
B(mdl, 'theta sel', 'simulink/Signal Routing/Selector', [1060 195 1080 225], 'Indices', '1', 'InputPortWidth', '4');
B(mdl, 'Scope mux', 'simulink/Signal Routing/Mux', [1170 180 1175 260], 'Inputs', '3');
B(mdl, 'Scope', 'simulink/Sinks/Scope', [1210 200 1250 240]);
L(mdl, 'Body Dynamics/1', 'theta sel/1'); L(mdl, 'theta sel/1', 'rad2deg/1');
L(mdl, 'rad2deg/1', 'Scope mux/1'); L(mdl, 'LQR Controller/1', 'Scope mux/2'); L(mdl, 'Gearmotor L/2', 'Scope mux/3');

note = Simulink.Annotation([mdl '/NanoBalancer encoder model']);
note.Text = sprintf(['NanoBalancer with Simscape gearmotors (Pololu 30:1 HPCB, 29.86:1) + MP6522 0.5 A + 12 CPR magnetic encoders\n' ...
    'EncoderMode: 1 true wheel state | 2 encoder counts (358.3 counts / wheel rev) | 3 encoder + Kalman observer\n' ...
    'SensorMode: 1 ideal | 2 one-sample delay | 3 one-sample delay + predictor | 4 two-sample delay + predictor']);
note.Position = [30 520 900 570];
Simulink.BlockDiagram.arrangeSystem(S);
save_system(mdl, f); fprintf('Built %s\n', f);
