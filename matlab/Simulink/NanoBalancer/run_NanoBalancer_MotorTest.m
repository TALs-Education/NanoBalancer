% run_NanoBalancer_MotorTest — validates the Simscape gearmotor against the
% Pololu #3049 datasheet and the linear model used for the LQR design.
here = fileparts(mfilename('fullpath')); cd(here);
mdl = 'NanoBalancer_MotorTest'; load_system(mdl);
test_u = 12; test_T = 2.5; test_wmax = 130;
cases = {'datasheet (no current limit)', 100; 'Nano Motor Carrier (0.5 A limit)', 0.5};
figure('Color','w','Position',[100 100 1000 420]);
for c = 1:2
    GM_Ilim = cases{c,2}; InitNanoBalancer; %#ok<NASGU>
    so = sim(mdl, 'SrcWorkspace', 'current');
    t = so.log_tau.time; tau = squeeze(so.log_tau.signals.values);
    I = squeeze(so.log_I.signals.values); w = squeeze(so.log_w.signals.values);
    cnt = squeeze(so.log_cnt.signals.values);
    rpm = w*60/(2*pi);
    % linear reference model (Excel / LQR)
    Il = min(GM.Ilim, (test_u - NB.Ke*w)/NB.R); tau_lin = NB.Kt*(Il - 0.08);
    k0 = find(t > 0.02, 1);                  % skip start-up transient
    inl = k0 - 1 + find(tau(k0:end) <= 0, 1); if isempty(inl), inl = numel(w); end
    w_nl = w(inl);
    fprintf('\n== %s\n', cases{c,1});
    fprintf('  stall torque  : %.4f N m (%.3f kg cm), current %.3f A\n', tau(k0), tau(k0)/0.0980665, I(k0));
    fprintf('  no-load speed : %.0f rpm, current at tau=0: %.3f A\n', w_nl*60/(2*pi), I(inl));
    fprintf('  max |tau - linear model| : %.2f mN m\n', 1e3*max(abs(tau(k0:end) - tau_lin(k0:end))));
    cnt_exp = trapz(t, w)*GM.N*GM.CPR/(2*pi);
    fprintf('  encoder count : %d  (expected %.0f from speed integral, error %.2f%%)\n', round(cnt(end)), cnt_exp, 100*(cnt(end)-cnt_exp)/cnt_exp);
    subplot(1,2,1); hold on; plot(rpm, tau/0.0980665); plot(rpm, tau_lin/0.0980665, '--');
    subplot(1,2,2); hold on; plot(rpm, I);
end
subplot(1,2,1); grid on; xlabel('output speed [rpm]'); ylabel('torque [kg cm]'); title('Torque-speed @ 12 V');
legend('Simscape, no limit', 'linear, no limit', 'Simscape, 0.5 A', 'linear, 0.5 A'); xlim([0 1200]); ylim([-0.05 0.45]);
subplot(1,2,2); grid on; xlabel('output speed [rpm]'); ylabel('current [A]'); title('Motor current'); xlim([0 1200]);
legend('no limit', '0.5 A limit');
saveas(gcf, fullfile(here, 'NanoBalancer_motor_validation.png'));
