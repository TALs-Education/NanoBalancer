% check_predictor — no bump, 3 deg initial tilt: with friction feed-forward the
% predictor modes should track the ideal-sensor response closely.
cd(fileparts(mfilename('fullpath')));
mdl = 'NanoBalancer_Stabilizer'; load_system(mdl);
theta0_deg = 3; bump_F = 0; bump_t = 1; bump_dt = 0.02;
for FrictionComp = [0 1]
    fprintf('\nFrictionComp = %d\n', FrictionComp);
    X = cell(1,4);
    for m = 1:4
        SensorMode = m; InitNanoBalancer; %#ok<NASGU>
        so = sim(mdl, 'SrcWorkspace', 'current', 'StopTime', '3');
        x = squeeze(so.log_x.signals.values); if size(x,1) ~= 4, x = x.'; end
        X{m} = interp1(so.log_x.time, x.', (0:0.001:3)').';
        fprintf('  mode %d: undershoot %.3f deg, peak travel %.1f mm, theta(1 s) %+.3f deg\n', m, ...
            -min(rad2deg(X{m}(1,:))), 1e3*max(abs(X{m}(3,:))), rad2deg(X{m}(1,1001)));
    end
    for m = 2:4
        fprintf('  max |theta - theta_ideal| mode %d: %.3f deg\n', m, max(abs(rad2deg(X{m}(1,:)-X{1}(1,:)))));
    end
end
