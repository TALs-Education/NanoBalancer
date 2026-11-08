% InitNanoBalancer — start-up script for NanoBalancer_Stabilizer.slx
% Loads the model/LQR parameters produced by matlab/NanoBalancer_LQR.m
% (struct NB in NanoBalancer_params.mat) and defines simulation settings.

here = fileparts(mfilename('fullpath'));
pfile = fullfile(here, '..', '..', 'NanoBalancer_params.mat');
if ~isfile(pfile)
    run(fullfile(here, '..', '..', 'NanoBalancer_LQR.m'));   % generates the .mat
end
load(pfile, 'NB');

sampleTime = NB.Ts;            % controller / BNO055 rate (0.01 s = 100 Hz)

% Sensor / controller setting (Multiport Switch "Sensor Mode"):
%   1 = ideal sensor (no delay)
%   2 = 1-sample BNO055 delay, no predictor
%   3 = 1-sample delay + model predictor
%   4 = 2-sample delay + model predictor
if ~exist('SensorMode', 'var'), SensorMode = 1; end
% Gearmotor friction feed-forward in the controller (1 = on, 0 = off)
if ~exist('FrictionComp', 'var'), FrictionComp = 1; end

% Plant parameter vector for the MATLAB Function plant block
%      [m_b    m_w    l    Ib    Iw    r    g    Kt    Ke    R    Ilim    Inl]
NB_pp = [NB.m_b NB.m_w NB.l NB.Ib NB.Iw NB.r NB.g NB.Kt NB.Ke NB.R NB.Ilim 0.08];

% Initial condition and bump (horizontal force pulse at the body CoM)
if ~exist('theta0_deg', 'var'), theta0_deg = 0;    end   % initial tilt [deg]
if ~exist('bump_F',     'var'), bump_F     = 2.5;  end   % force [N]
if ~exist('bump_t',     'var'), bump_t     = 1.0;  end   % start time [s]
if ~exist('bump_dt',    'var'), bump_dt    = 0.02; end   % duration [s] -> impulse = F*dt
x0 = [deg2rad(theta0_deg); 0; 0; 0];                     % [theta; theta_dot; x; x_dot]

%% Simscape gearmotor + encoder (NanoBalancer_Encoder.slx, NanoBalancer_MotorTest.slx)
% Pololu #3049 30:1 Micro Metal Gearmotor HPCB 12 V, values referred to the MOTOR shaft.
% Derived from the datasheet linear fit used in the LQR design (NB.Kt, NB.Ke, NB.R).
GM.N        = 29.86;              % gear ratio
GM.R        = NB.R;               % armature resistance [ohm] (12 V / 0.75 A)
GM.Ke       = NB.Ke/GM.N;         % back-EMF = torque constant at motor shaft [V s/rad]
GM.La       = 0.3e-3;             % armature inductance [H]  (ASSUMPTION, typical 10 mm motor)
GM.J_rotor  = 3e-9;               % rotor inertia [kg m^2]   (ASSUMPTION, 0.03 g cm^2)
GM.Tc       = GM.Ke*0.08;         % Coulomb friction at motor shaft -> 0.08 A no-load current
GM.eta_gb   = NB.Kt/NB.Ke;        % gearbox efficiency (stall torque / ideal) ~0.61
GM.Vmax     = NB.Vmax;            % Nano Motor Carrier boosted motor supply [V]
if ~exist('GM_Ilim', 'var'), GM_Ilim = NB.Ilim; end
GM.Ilim     = GM_Ilim;            % MP6522 current regulation per channel [A]
GM.PPR      = 3;                  % magnetic encoder pulses per channel per motor rev
GM.CPR      = 4*GM.PPR;           % counts per motor rev (x4 decoding) = 12
GM.cnt2rad  = 2*pi/(GM.CPR*GM.N); % wheel angle per count [rad] (358.3 counts / wheel rev)
GM.tau_sign = 1;                  % torque sensor sign (validated in NanoBalancer_MotorTest)

% Body-only plant for the Simscape version (rotor inertia lives in the Simscape motor)
Iw_wheels = NB.Iw - 2*GM.J_rotor*GM.N^2;
%        [m_b    m_w    l    Ib    Iw_wheels  r    g]
NB_pb = [NB.m_b NB.m_w NB.l NB.Ib Iw_wheels NB.r NB.g];
% EncoderMode: 1 = true wheel state, 2 = encoder count difference, 3 = encoder + Kalman observer
if ~exist('EncoderMode', 'var'), EncoderMode = 3; end
% Encoder speed low-pass (first order, discrete) for EncoderMode 2. 0 = raw count difference.
if ~exist('enc_tau', 'var'), enc_tau = 0; end            % [s]
enc_a = NB.Ts/(NB.Ts + enc_tau);

%% Steady-state Kalman observer (EncoderMode 3)
% Measurements y = [theta (BNO055); theta_dot (BNO055 gyro); x (encoders)]
KF_C  = [1 0 0 0; 0 1 0 0; 0 0 1 0];
kf_sig_thdd = 20;                  % process noise: unmodelled tilt acceleration [rad/s^2]
kf_sig_xdd  = 2;                   % process noise: unmodelled wheel acceleration [m/s^2]
kf_sig_th   = deg2rad(0.2);        % BNO055 fusion angle noise [rad]   (ASSUMPTION)
kf_sig_thd  = deg2rad(1.0);        % BNO055 gyro noise [rad/s]          (ASSUMPTION)
kf_sig_x    = NB.r*GM.cnt2rad/sqrt(12);   % encoder quantisation (0.35 mm / sqrt(12)) [m]
Ts_ = NB.Ts;
Gw = [Ts_^2/2 0; Ts_ 0; 0 Ts_^2/2; 0 Ts_];          % acceleration noise -> state
KF_Q = Gw*diag([kf_sig_thdd^2, kf_sig_xdd^2])*Gw.';
KF_R = diag([kf_sig_th^2, kf_sig_thd^2, kf_sig_x^2]);
Pk = eye(4)*1e-3;
for it = 1:5000                                      % discrete Riccati (filter form)
    Pm = NB.Ad*Pk*NB.Ad.' + KF_Q;
    KF_L = Pm*KF_C.'/(KF_C*Pm*KF_C.' + KF_R);
    Pn = (eye(4) - KF_L*KF_C)*Pm;
    if norm(Pn - Pk, 'fro') < 1e-14, Pk = Pn; break; end
    Pk = Pn;
end
KF_Ad = NB.Ad; KF_Bd = NB.Bd;
KF_ff = [FrictionComp*NB.R*0.08, NB.r];               % friction FF model used by the observer
