% rebuild_all — regenerate the NanoBalancer parameters and Simulink models in the
% running MATLAB release, then re-run every validation script.
% Usage (from any folder):  run('<project>/matlab/Simulink/NanoBalancer/rebuild_all.m')
here = fileparts(mfilename('fullpath'));
addpath(here);                       % survives the 'clear' inside NanoBalancer_LQR.m
fprintf('==== NanoBalancer rebuild in MATLAB %s ====\n', version);
bdclose('all');
% remove simulation caches from older releases
if isfolder(fullfile(here, 'slprj')), rmdir(fullfile(here, 'slprj'), 's'); end
delete(fullfile(here, '*.slxc'));

t0 = tic;
fprintf('\n---- 1) LQR design + parameters ----\n');
run(fullfile(here, '..', '..', 'NanoBalancer_LQR.m')); close all;
here = fileparts(which('rebuild_all')); t0 = tic;
cd(here);
fprintf('\n---- 2) build models ----\n');
build_NanoBalancer_Stabilizer; bdclose('all');
build_NanoBalancer_MotorTest;  bdclose('all');
build_NanoBalancer_Encoder;    bdclose('all');
for m = {'NanoBalancer_Stabilizer', 'NanoBalancer_MotorTest', 'NanoBalancer_Encoder'}
    info = Simulink.MDLInfo(fullfile(here, [m{1} '.slx']));
    fprintf('  %-26s saved in %s\n', m{1}, info.ReleaseName);
end
fprintf('\n---- 3) validations ----\n');
clearvars -except here t0
run_NanoBalancer_MotorTest;    close all; bdclose('all'); clearvars -except here t0
run_NanoBalancer_Stabilizer;   close all; bdclose('all'); clearvars -except here t0
check_predictor;               close all; bdclose('all'); clearvars -except here t0
run_NanoBalancer_Encoder;      close all; bdclose('all');
fprintf('\n==== done in %.0f s ====\n', toc(t0));
