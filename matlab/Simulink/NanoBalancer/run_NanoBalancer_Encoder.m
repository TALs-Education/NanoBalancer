% run_NanoBalancer_Encoder — bump response with Simscape motors + encoders
here = fileparts(mfilename('fullpath')); cd(here);
mdl = 'NanoBalancer_Encoder'; load_system(mdl);
theta0_deg = 0; bump_F = 2.5; bump_t = 1.0; bump_dt = 0.02; FrictionComp = 1;
%        label                                SensorMode EncoderMode enc_tau
%        label                                    SensorMode EncoderMode enc_tau
runs = {'Ideal wheel state, SensorMode 1',                1, 1, 0;
        'Encoder count difference, SensorMode 1',        1, 2, 0;
        'Encoder diff + 30 ms LPF, SensorMode 1',        1, 2, 0.03;
        'Encoder + Kalman observer, SensorMode 1',       1, 3, 0;
        'Encoder + Kalman, SensorMode 3 (1-step pred)',  3, 3, 0};
R = struct([]);
for k = 1:size(runs,1)
    SensorMode = runs{k,2}; EncoderMode = runs{k,3}; enc_tau = runs{k,4}; InitNanoBalancer; %#ok<NASGU>
    tic; so = sim(mdl, 'SrcWorkspace', 'current'); el = toc;
    x = squeeze(so.log_x.signals.values); if size(x,1) ~= 4, x = x.'; end
    xm = squeeze(so.log_xm.signals.values); if size(xm,1) ~= 4, xm = xm.'; end
    R(k).t = so.log_x.time; R(k).x = x; R(k).tm = so.log_xm.time; R(k).xm = xm;
    R(k).tu = so.log_u.time; R(k).u = squeeze(so.log_u.signals.values);
    R(k).tI = so.log_I.time; R(k).I = squeeze(so.log_I.signals.values);
    R(k).cnt = squeeze(so.log_cnt.signals.values);
    after = R(k).tu > 2.5;
    fprintf('%-40s peak tilt %5.2f deg | peak travel %5.1f mm | peak |u| %4.1f V | peak |I| %.2f A | u rms after 2.5 s %.2f V | counts end %d | %.0f s\n', ...
        runs{k,1}, max(abs(rad2deg(x(1,:)))), 1e3*max(abs(x(3,:))), max(abs(R(k).u)), max(abs(R(k).I)), ...
        rms(R(k).u(after)), round(R(k).cnt(end)), el);
end
figure('Name','NanoBalancer encoder model','Color','w','Position',[60 60 1150 780]);
ax = gobjects(4,1); ttl = {'Body tilt \theta [deg]', 'Wheel travel x [mm] (true)', 'Voltage command u [V]', 'Motor current (left) [A]'};
for k = 1:size(runs,1)
    ax(1) = subplot(2,2,1); hold on; plot(R(k).t, rad2deg(R(k).x(1,:)));
    ax(2) = subplot(2,2,2); hold on; plot(R(k).t, 1e3*R(k).x(3,:));
    ax(3) = subplot(2,2,3); hold on; stairs(R(k).tu, R(k).u);
    ax(4) = subplot(2,2,4); hold on; plot(R(k).tI, R(k).I);
end
for k = 1:4, grid(ax(k),'on'); title(ax(k), ttl{k}); xlabel(ax(k),'t [s]'); end
legend(ax(1), runs(:,1), 'Location', 'best');
sgtitle('Bump 0.05 N\cdots, Simscape gearmotors + 12 CPR encoders, 100 Hz LQR');
saveas(gcf, fullfile(here, 'NanoBalancer_encoder_bump.png'));

% measured vs true wheel speed (encoder quantisation)
figure('Color','w','Position',[80 80 900 350]);
plot(R(4).t, R(4).x(4,:)*1e3); hold on; stairs(R(4).tm, R(4).xm(4,:)*1e3, 'LineWidth', 1.2); grid on;
xlabel('t [s]'); ylabel('x dot [mm/s]'); legend('true', 'Kalman estimate @100 Hz'); title('Wheel speed: true vs Kalman estimate from encoder + IMU');
saveas(gcf, fullfile(here, 'NanoBalancer_encoder_speed.png'));
