% build_NanoBalancer_MotorTest — test bench for the Simscape gearmotor subsystem:
% constant 12 V command, output speed swept 0 -> above no-load speed.
here = fileparts(mfilename('fullpath')); cd(here);
mdl = 'NanoBalancer_MotorTest';
if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']); if isfile(f), delete(f); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'PreLoadFcn', 'InitNanoBalancer', 'InitFcn', 'InitNanoBalancer', ...
    'StopTime', 'test_T', 'Solver', 'daessc', 'MaxStep', '2e-5', 'RelTol', '1e-4');
add_block('simulink/Sources/Constant', [mdl '/u = 12 V'], 'Position', [30 40 80 70], 'Value', 'test_u');
add_block('simulink/Sources/Ramp', [mdl '/w_rel sweep'], 'Position', [30 120 80 150], ...
    'Slope', 'test_wmax/test_T', 'Start', '0', 'InitialOutput', '0');
nb_add_gearmotor(mdl, 'Gearmotor', [160 30 300 170]);
add_line(mdl, 'u = 12 V/1', 'Gearmotor/1'); add_line(mdl, 'w_rel sweep/1', 'Gearmotor/2');
tw = {'SaveFormat', 'Structure With Time'};
add_block('simulink/Sinks/To Workspace', [mdl '/log_tau'], 'Position', [380 30 440 50], 'VariableName', 'log_tau', tw{:});
add_block('simulink/Sinks/To Workspace', [mdl '/log_I'], 'Position', [380 80 440 100], 'VariableName', 'log_I', tw{:});
add_block('simulink/Sinks/To Workspace', [mdl '/log_cnt'], 'Position', [380 130 440 150], 'VariableName', 'log_cnt', tw{:});
add_block('simulink/Sinks/To Workspace', [mdl '/log_w'], 'Position', [160 200 220 220], 'VariableName', 'log_w', tw{:});
add_line(mdl, 'Gearmotor/1', 'log_tau/1'); add_line(mdl, 'Gearmotor/2', 'log_I/1');
add_line(mdl, 'Gearmotor/3', 'log_cnt/1'); add_line(mdl, 'w_rel sweep/1', 'log_w/1');
save_system(mdl, f); fprintf('Built %s\n', f);
