% run_NanoBalancer_Stabilizer — bump response of all four sensor settings
% and the largest bump (impulse) each setting survives.
here = fileparts(mfilename('fullpath')); cd(here);
mdl = 'NanoBalancer_Stabilizer';
load_system(mdl);
names = {'1 ideal', '2 one-sample delay', '3 one-sample delay + predictor', '4 two-sample delay + predictor'};

% ---- nominal bump response ----
theta0_deg = 0; bump_F = 2.5; bump_t = 1.0; bump_dt = 0.02;
R = struct([]);
for m = 1:4
    SensorMode = m; %#ok<NASGU>
    InitNanoBalancer;
    so = sim(mdl, 'SrcWorkspace', 'current', 'StopTime', '4');
    R(m).t = so.log_x.time; R(m).x = squeeze(so.log_x.signals.values);
    if size(R(m).x,1) ~= 4, R(m).x = R(m).x.'; end
    R(m).tu = so.log_u.time; R(m).u = squeeze(so.log_u.signals.values);
    R(m).I = squeeze(so.log_I.signals.values); R(m).tI = so.log_I.time;
    fprintf('%-34s peak tilt %5.2f deg, peak travel %5.1f mm, peak |u| %4.1f V, peak |I| %.2f A\n', names{m}, ...
        max(abs(rad2deg(R(m).x(1,:)))), 1e3*max(abs(R(m).x(3,:))), max(abs(R(m).u)), max(abs(R(m).I)));
end

figure('Name','NanoBalancer bump response','Color','w','Position',[80 80 1100 750]);
ax = gobjects(4,1);
for m = 1:4
    ax(1) = subplot(2,2,1); hold on; plot(R(m).t, rad2deg(R(m).x(1,:)));
    ax(2) = subplot(2,2,2); hold on; plot(R(m).t, 1e3*R(m).x(3,:));
    ax(3) = subplot(2,2,3); hold on; stairs(R(m).tu, R(m).u);
    ax(4) = subplot(2,2,4); hold on; plot(R(m).tI, R(m).I);
end
ttl = {'Body tilt \theta [deg]', 'Wheel travel x [mm]', 'Voltage command u [V]', 'Motor current [A] (limit 0.5)'};
for k = 1:4, grid(ax(k),'on'); title(ax(k), ttl{k}); xlabel(ax(k),'t [s]'); end
legend(ax(1), names, 'Location', 'best');
sgtitle(sprintf('Bump %.2f N\\cdots at body CoM, t = %.1f s, 100 Hz LQR', bump_F*bump_dt, bump_t));
saveas(gcf, fullfile(here, 'NanoBalancer_bump_response.png'));

% ---- largest survivable bump impulse per mode ----
fprintf('\nLargest bump impulse recovered (force pulse %.0f ms at CoM):\n', bump_dt*1e3);
for m = 1:4
    lo = 0; hi = 40;      % force [N]
    for it = 1:12
        bump_F = (lo+hi)/2; SensorMode = m; InitNanoBalancer;
        so = sim(mdl, 'SrcWorkspace', 'current', 'StopTime', '4');
        th = squeeze(so.log_x.signals.values); if size(th,1) ~= 4, th = th.'; end
        ok = all(abs(th(1,:)) < deg2rad(80)) && abs(th(1,end)) < deg2rad(1);
        if ok, lo = bump_F; else, hi = bump_F; end
    end
    bump_F = lo; InitNanoBalancer;
    so = sim(mdl, 'SrcWorkspace', 'current', 'StopTime', '4');
    th = squeeze(so.log_x.signals.values); if size(th,1) ~= 4, th = th.'; end
    fprintf('  %-34s %.3f N s  (%.1f N) -> peak tilt %.1f deg\n', names{m}, lo*bump_dt, lo, max(abs(rad2deg(th(1,:)))));
end
