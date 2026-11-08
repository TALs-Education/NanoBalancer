function blk = nb_add_gearmotor(sys, name, pos)
% nb_add_gearmotor — adds a "Pololu 30:1 HPCB gearmotor + MP6522 driver +
% magnetic encoder" subsystem to SYS.
%
%   Inports : 1 u [V]        voltage command (Nano Motor Carrier, |u| <= 12 V)
%             2 w_rel [rad/s] gearbox output speed relative to the body (stator)
%   Outports: 1 tau [N m]    torque delivered by the gearbox output to the wheel
%             2 I [A]        motor current
%             3 count        quadrature encoder count (12 CPR on the motor shaft)
%
% Parameters come from the workspace struct GM (see InitNanoBalancer.m).

blk = [sys '/' name];
add_block('simulink/Ports & Subsystems/Subsystem', blk, 'Position', pos);
delete_line(blk, 'In1/1', 'Out1/1'); delete_block([blk '/In1']); delete_block([blk '/Out1']);

A  = @(n, src, p, varargin) add_block(src, [blk '/' n], 'Position', p, varargin{:});
SL = @(a, b) add_line(blk, a, b, 'autorouting', 'smart');
H  = @(n) get_param([blk '/' n], 'PortHandles');
PL = @(h1, h2) add_line(blk, h1, h2, 'autorouting', 'smart');

%% Simulink side: ports + MP6522 current regulation (averaged)
A('u [V]',     'simulink/Sources/In1',  [20 40 50 55]);
A('w_rel',     'simulink/Sources/In1',  [20 160 50 175], 'Port', '2');
A('tau [Nm]',  'simulink/Sinks/Out1',   [1100 300 1130 315]);
A('I [A]',     'simulink/Sinks/Out1',   [1100 80 1130 95],  'Port', '2');
A('count',     'simulink/Sinks/Out1',   [1140 480 1170 495], 'Port', '3');
A('Vsupply clamp', 'simulink/Discontinuities/Saturation', [90 35 130 60], ...
  'UpperLimit', 'GM.Vmax', 'LowerLimit', '-GM.Vmax');
A('back-EMF', 'simulink/Math Operations/Gain', [90 155 150 180], 'Gain', 'GM.N*GM.Ke');
A('+R*Ilim', 'simulink/Math Operations/Bias', [190 110 240 135], 'Bias', 'GM.R*GM.Ilim');
A('-R*Ilim', 'simulink/Math Operations/Bias', [190 190 240 215], 'Bias', '-GM.R*GM.Ilim');
A('MP6522 current regulation', 'simulink/Discontinuities/Saturation Dynamic', [290 30 340 220]);
SL('u [V]/1', 'Vsupply clamp/1');
SL('w_rel/1', 'back-EMF/1');
SL('back-EMF/1', '+R*Ilim/1'); SL('back-EMF/1', '-R*Ilim/1');
SL('+R*Ilim/1', 'MP6522 current regulation/1');
SL('Vsupply clamp/1', 'MP6522 current regulation/2');
SL('-R*Ilim/1', 'MP6522 current regulation/3');

%% Simscape physical network
A('V to PS', 'nesl_utility/Simulink-PS Converter', [390 115 420 135]);
A('w to PS', 'nesl_utility/Simulink-PS Converter', [390 395 420 415], ...
  'FilteringAndDerivatives', 'filter', 'SimscapeFilterOrder', '2', 'InputFilterTimeConstant', '2e-4');
A('Driver output', 'fl_lib/Electrical/Electrical Sources/Controlled Voltage Source', [470 100 510 150]);
A('Current Sensor', 'fl_lib/Electrical/Electrical Sensors/Current Sensor', [560 40 600 80]);
A('I to SL', 'nesl_utility/PS-Simulink Converter', [660 80 690 100]);
A('DC Motor 30:1 HPCB', 'ee_lib/Electromechanical/Brushed Motors/DC Motor', [650 140 720 210], ...
  'parameterization', 'ee.enum.dcparam_option.circuit_param', ...
  'Ra', 'GM.R', 'La', 'GM.La', 'La_unit', 'H', ...
  'emf_or_torque', 'ee.enum.emf_or_torque.kv', 'Kv', 'GM.Ke', 'Kv_unit', 'V/(rad/s)', 'Ra_unit', 'Ohm', ...
  'lam_param', 'ee.enum.lam_param.damping', 'lam', '0', ...
  'J', 'GM.J_rotor', 'J_unit', 'kg*m^2');
A('ERef', 'fl_lib/Electrical/Electrical Elements/Electrical Reference', [470 220 500 250]);
A('Motor shaft friction', 'fl_lib/Mechanical/Rotational Elements/Rotational Friction', [780 210 840 250], ...
  'brkwy_trq', 'GM.Tc', 'Col_trq', 'GM.Tc', 'brkwy_vel', '1', 'visc_coef', '0');
A('Gearbox 29.86:1', 'sdl_lib/Gears/Simple Gear', [780 140 840 190], ...
  'ratio', 'GM.N', 'rotation_direction', '1', ...
  'friction_model', 'sdl.enum.gear_efficiencies_load.constant', 'efficiency', 'GM.eta_gb', ...
  'w_thr', '0.5');
A('Output torque sensor', 'fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor', [880 280 920 320]);
A('tau to SL', 'nesl_utility/PS-Simulink Converter', [960 300 990 320]);
A('Wheel speed rel. body', 'fl_lib/Mechanical/Mechanical Sources/Ideal Angular Velocity Source', [880 380 920 420]);
A('MRRef', 'fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference', [800 440 830 470]);
A('Solver Configuration', 'nesl_utility/Solver Configuration', [560 260 610 290]);
% magnetic encoder on the motor shaft
A('Magnetic encoder 12 CPR', 'ee_lib/Sensors & Transducers/Incremental Shaft Encoder', [650 470 720 540], ...
  'NPulse', 'GM.PPR', 'Vm', '3.3', 'angle0', '1', 'angle0_unit', 'deg');
A('ERef enc', 'fl_lib/Electrical/Electrical Elements/Electrical Reference', [760 570 790 600]);
A('A sense', 'fl_lib/Electrical/Electrical Sensors/Voltage Sensor', [770 460 810 500]);
A('B sense', 'fl_lib/Electrical/Electrical Sensors/Voltage Sensor', [770 510 810 550]);
A('A to SL', 'nesl_utility/PS-Simulink Converter', [840 470 870 490]);
A('B to SL', 'nesl_utility/PS-Simulink Converter', [840 520 870 540]);

SL('MP6522 current regulation/1', 'V to PS/1');
SL('w_rel/1', 'w to PS/1');
h = @(n) H(n);
PL(h('V to PS').RConn(1), h('Driver output').RConn(1));          % PS input
PL(h('Driver output').LConn(1), h('Current Sensor').LConn(1));   % + -> sensor +
PL(h('Current Sensor').RConn(2), h('DC Motor 30:1 HPCB').LConn(1)); % sensor - -> motor +
PL(h('Current Sensor').RConn(1), h('I to SL').LConn(1));         % I signal
PL(h('DC Motor 30:1 HPCB').RConn(1), h('Driver output').RConn(2)); % motor - -> source -
PL(h('Driver output').RConn(2), h('ERef').LConn(1));
PL(h('ERef').LConn(1), h('Solver Configuration').RConn(1));
% mechanical: rotor -> gearbox base, friction to case
PL(h('DC Motor 30:1 HPCB').LConn(2), h('Gearbox 29.86:1').LConn(1));   % shaft
PL(h('DC Motor 30:1 HPCB').LConn(2), h('Motor shaft friction').LConn(1));
PL(h('Motor shaft friction').RConn(1), h('MRRef').LConn(1));
PL(h('DC Motor 30:1 HPCB').RConn(2), h('MRRef').LConn(1));
% gearbox follower -> torque sensor -> velocity source (wheel relative to body)
PL(h('Gearbox 29.86:1').RConn(1), h('Output torque sensor').LConn(1));
PL(h('Output torque sensor').RConn(1), h('Wheel speed rel. body').LConn(1));
PL(h('Wheel speed rel. body').RConn(2), h('MRRef').LConn(1));
PL(h('w to PS').RConn(1), h('Wheel speed rel. body').RConn(1));
PL(h('Output torque sensor').RConn(2), h('tau to SL').LConn(1));
% encoder on rotor shaft
PL(h('DC Motor 30:1 HPCB').LConn(2), h('Magnetic encoder 12 CPR').LConn(1));
PL(h('Magnetic encoder 12 CPR').LConn(2), h('MRRef').LConn(1));
PL(h('Magnetic encoder 12 CPR').RConn(4), h('ERef enc').LConn(1));
PL(h('Magnetic encoder 12 CPR').RConn(1), h('A sense').LConn(1));
PL(h('Magnetic encoder 12 CPR').RConn(2), h('B sense').LConn(1));
PL(h('A sense').RConn(2), h('ERef enc').LConn(1));
PL(h('B sense').RConn(2), h('ERef enc').LConn(1));
PL(h('A sense').RConn(1), h('A to SL').LConn(1));
PL(h('B sense').RConn(1), h('B to SL').LConn(1));

%% torque sign + outputs
A('tau sign', 'simulink/Math Operations/Gain', [1020 300 1060 320], 'Gain', 'GM.tau_sign');
SL('tau to SL/1', 'tau sign/1'); SL('tau sign/1', 'tau [Nm]/1');
SL('I to SL/1', 'I [A]/1');

%% quadrature decoder (x4) — event driven, like a hardware counter
A('A logic', 'simulink/Logic and Bit Operations/Compare To Constant', [900 465 950 495], 'relop', '>', 'const', '1.65');
A('B logic', 'simulink/Logic and Bit Operations/Compare To Constant', [900 515 950 545], 'relop', '>', 'const', '1.65');
SL('A to SL/1', 'A logic/1'); SL('B to SL/1', 'B logic/1');
A('AB mux', 'simulink/Signal Routing/Mux', [980 460 985 550], 'Inputs', '2');
SL('A logic/1', 'AB mux/1'); SL('B logic/1', 'AB mux/2');
Q = [blk '/Quadrature decoder x4'];
add_block('simulink/Ports & Subsystems/Triggered Subsystem', Q, 'Position', [1010 460 1070 520]);
set_param([Q '/Trigger'], 'TriggerType', 'either');
delete_line(Q, 'In1/1', 'Out1/1');
add_block('simulink/User-Defined Functions/MATLAB Function', [Q '/decode'], 'Position', [150 40 260 90]);
code = strjoin({
'function cnt = decode(AB)'
'% x4 quadrature decoding: +1 per edge when A leads B (positive rotation)'
'persistent prev c'
'if isempty(prev), prev = 2*double(AB(1)) + double(AB(2)); c = 0; end'
's = 2*double(AB(1)) + double(AB(2));'
'T = [0 -1 1 0; 1 0 0 -1; -1 0 0 1; 0 1 -1 0];'
'c = c + T(prev+1, s+1);'
'prev = s;'
'cnt = c;'
}, newline);
rt = sfroot; ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [Q '/decode']); ch.Script = code;
d = ch.find('-isa', 'Stateflow.Data', 'Name', 'AB'); d.Props.Array.Size = '[2 1]';
add_line(Q, 'In1/1', 'decode/1', 'autorouting', 'smart');
add_line(Q, 'decode/1', 'Out1/1', 'autorouting', 'smart');
set_param([Q '/Out1'], 'InitialOutput', '0');
SL('AB mux/1', 'Quadrature decoder x4/1');
SL('AB mux/1', 'Quadrature decoder x4/Trigger');
A('count hold', 'simulink/Discrete/Memory', [1080 475 1100 500]);   % breaks the trigger loop (one solver step)
SL('Quadrature decoder x4/1', 'count hold/1');
SL('count hold/1', 'count/1');
end
