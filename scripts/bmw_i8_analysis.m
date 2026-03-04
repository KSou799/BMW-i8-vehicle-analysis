% =========================================================================
% BMW i8 VEHICLE PERFORMANCE ANALYSIS
% =========================================================================

clear all; close all; clc;

fprintf('========================================\n');
fprintf(' BMW i8 VEHICLE PERFORMANCE ANALYSIS\n');
fprintf('         — CORRECTED MODEL —\n');
fprintf('========================================\n\n');

%% LOAD VEHICLE DATA
fprintf('Loading vehicle data...\n');
run('VehicleData_fixed')
fprintf('  ✓ Loaded (hybrid powertrain: ICE 170 kW rear + EM 96 kW front)\n\n');

% =========================================================================
%% 1. AERODYNAMIC DRAG ANALYSIS
% =========================================================================
fprintf('1. Aerodynamic Drag Analysis...\n');

Fd = rho/2 * cd * A * (vT/3.6).^2;   % Drag force [N]
Pd = Fd .* (vT/3.6);                   % Power [W]

figure('Name','1. Aerodynamic Drag Analysis','NumberTitle','off')
subplot(1,2,1)
  plot(vT, Fd, 'b', 'LineWidth', 2)
  title('Aerodynamic Drag Force vs. Velocity', 'FontWeight','bold')
  xlabel('Velocity [km/h]', 'FontWeight','bold')
  ylabel('Force [N]',       'FontWeight','bold')
  grid on

subplot(1,2,2)
  plot(vT, Pd/1000, 'r', 'LineWidth', 2)
  title('Aerodynamic Drag Power vs. Velocity', 'FontWeight','bold')
  xlabel('Velocity [km/h]', 'FontWeight','bold')
  ylabel('Power [kW]',      'FontWeight','bold')
  grid on

fprintf('  ✓ Complete\n\n');

% =========================================================================
%% 2. ROLLING RESISTANCE ANALYSIS (level road, static load distribution)
% =========================================================================
fprintf('2. Rolling Resistance Analysis...\n');

% Static axle loads — single wheel
Gtf = G * sr / l / 2;   % Front wheel load [N]  (sr/l = front load fraction)
Gtr = G * sf / l / 2;   % Rear  wheel load [N]  (sf/l = rear  load fraction)

% "Real" rolling resistance coefficients (Mitschke/Reimpell formula)
% Pressure term = 1 when p_actual = p_nominal (both axles here)
frf = fR * (1.3 - 0.3*pf/pTf) * (1.3 - 0.3*NTf/Gtf);  % Front
frr = fR * (1.3 - 0.3*pr/pTr) * (1.3 - 0.3*NTr/Gtr);  % Rear

Ftf = frf * Gtf * 2;    % Front axle rolling resistance [N]
Ftr = frr * Gtr * 2;    % Rear  axle rolling resistance [N]
Ft  = Ftf + Ftr;        % Total rolling resistance [N]
Pt  = (vT/3.6) .* Ft;   % Rolling resistance power [W]

figure('Name','2. Rolling Resistance Analysis','NumberTitle','off')
subplot(1,2,1)
  plot(vT, Ft, 'b', 'LineWidth', 2)
  title('Rolling Resistance Force vs. Speed', 'FontWeight','bold')
  xlabel('Speed [km/h]', 'FontWeight','bold')
  ylabel('Force [N]',    'FontWeight','bold')
  grid on

subplot(1,2,2)
  plot(vT, Pt/1000, 'r', 'LineWidth', 2)
  title('Rolling Resistance Power vs. Speed', 'FontWeight','bold')
  xlabel('Speed [km/h]', 'FontWeight','bold')
  ylabel('Power [kW]',   'FontWeight','bold')
  grid on

fprintf('  ✓ Complete\n\n');

% =========================================================================
%% 3. GRADE RESISTANCE ANALYSIS
% =========================================================================
fprintf('3. Grade Resistance Analysis...\n');

Fg_grade = zeros(1, length(Grad));
Pg       = zeros(length(Grad), length(vT));

for i = 1:length(Grad)
  alpha         = atand(Grad(i)/100);
  Fg_grade(i)   = G * sind(alpha);
  Pg(i,:)       = (vT/3.6) * Fg_grade(i);
end

figure('Name','3. Grade Resistance Analysis','NumberTitle','off')
  surf(vT, Grad, Pg/1000)
  title('Power Required for Grade Climbing', 'FontWeight','bold')
  xlabel('Velocity [km/h]', 'FontWeight','bold')
  ylabel('Incline [%]',     'FontWeight','bold')
  zlabel('Power [kW]',      'FontWeight','bold')
  colorbar

fprintf('  ✓ Complete\n\n');

% =========================================================================
%% 4. TRACTIVE FORCE CALCULATION (CORRECTED: Hybrid powertrain + traction)
% =========================================================================
fprintf('4. Tractive Force Calculation (hybrid model)...\n');

n_grad = length(Grad);
n_v    = length(vT);
n_gear = length(i_g);
n_rpm  = length(nm_ice);

% --- 4a. Resistance forces for all gradients and speeds -----------------
Fd_trac = rho/2 * cd * A * (vT/3.6).^2;   % Aero drag [N]
FR      = zeros(n_grad, n_v);

for i = 1:n_grad
  alpha_i    = atand(Grad(i)/100);
  Fg_i       = G * sind(alpha_i);           % Grade resistance [N]
  fR_grade_i = fR * cosd(alpha_i);          % Rolling coeff corrected for slope
  Fr_i       = fR_grade_i * G;              % Approximate total rolling resistance [N]
  FR(i,:)    = Fd_trac + Fg_i + Fr_i;
end

% --- 4b. Static axle normal forces (for traction limit baseline) ---------
N_front_static = G * sr / l;   % Front axle [N]  (= 44.1 % of G)
N_rear_static  = G * sf / l;   % Rear  axle [N]  (= 55.9 % of G)

% --- 4c. ICE tractive force per gear (over nm_ice RPM range) -------------
%  FIX: uses Mm_ice (ICE-only torque, max 320 Nm), NOT combined 571 Nm.
%  Rear axle traction limit applied (static approximation).

v_ice  = zeros(n_gear, n_rpm);   % Wheel speed for each gear [km/h]
F_ice  = zeros(n_gear, n_rpm);   % ICE wheel force [N]

for g = 1:n_gear
  v_ice(g,:) = 2*pi * Rd * (nm_ice/60) / (i_f * i_g(g)) * 3.6;
  F_ice(g,:) = eta_ice * Mm_ice * i_f * i_g(g) / Rd;
  % Rear traction cap (static weight, conservative)
  F_ice(g,:) = min(F_ice(g,:), mu * N_rear_static);
end

% --- 4d. EM tractive force over full vT range (front axle) ---------------
%  Below v_em_base: constant torque T_em_peak.
%  Above v_em_base: power-limited  T = P_em / omega_motor.
%  Front axle traction cap applied.

F_em_vT = zeros(1, n_v);
for k = 1:n_v
  v_ms       = vT(k) / 3.6;
  omega_em_k = (v_ms / Rd_f) * i_em;          % Motor shaft speed [rad/s]
  if omega_em_k <= omega_em_base
    T_em_k = T_em_peak;                        % Constant-torque region
  else
    T_em_k = P_em / omega_em_k;               % Power-limited region
  end
  F_em_vT(k) = eta_em * T_em_k * i_em / Rd_f;
end
F_em_vT = min(F_em_vT, mu * N_front_static);  % Front traction cap

% --- 4e. Combined tractive force (ICE + EM) per gear --------------------
F_combined = zeros(n_gear, n_rpm);
for g = 1:n_gear
  % Interpolate EM force at speeds corresponding to this gear's RPM range
  F_em_interp    = interp1(vT, F_em_vT, v_ice(g,:), 'pchip', 'extrap');
  F_em_interp    = max(F_em_interp, 0);
  F_combined(g,:) = F_ice(g,:) + F_em_interp;
end

% --- 4f. Theoretical CVT envelope (educational comparison) ---------------
%  Represents maximum possible tractive force if a perfect CVT kept the
%  combined powertrain at peak power at all speeds.
%  NOTE: the i8 has NO CVT. This is a textbook upper-bound reference.
Fcvt = Pmax ./ (vT/3.6);

% --- 4g. Plot tractive force diagram ------------------------------------
figure('Name','4. Tractive Force Graph (Hybrid)','NumberTitle','off')
hold on

% Resistance curves (black) — label gradient at right end
for i = 1:n_grad
  plot(vT, FR(i,:), 'k', 'LineWidth', 0.8)
  grade_label = [' ' num2str(Grad(i)) '%'];
  text(vT(end), FR(i,end), grade_label, 'FontSize', 7)
end

% EM-only force (green) — shows front-axle electric contribution
plot(vT, F_em_vT, 'g-', 'LineWidth', 1.5, 'DisplayName', 'EM (front axle)')

% ICE-only per gear (blue dashed) — rear axle only
for g = 1:n_gear
  plot(v_ice(g,:), F_ice(g,:), 'b--', 'LineWidth', 1)
end

% Combined force per gear (red, bold) — what the car actually delivers
for g = 1:n_gear
  h_comb = plot(v_ice(g,:), F_combined(g,:), 'r-', 'LineWidth', 1.8);
  % Gear label at curve midpoint
  mid = round(n_rpm/2);
  text(v_ice(g,mid), F_combined(g,mid)+300, num2str(g), ...
       'Color','r', 'FontWeight','bold', 'FontSize', 9)
end

% CVT theoretical (dashed black)
plot(vT, Fcvt, 'k--', 'LineWidth', 1.5)
text(vT(20), Fcvt(20)+200, ' CVT (theoretical)', 'FontSize', 8)

% Annotations
text(10, 5600, 'EM', 'Color','g', 'FontWeight','bold')
text(10, 4800, 'ICE/gear', 'Color','b')
text(10, 4000, 'Combined', 'Color','r', 'FontWeight','bold')

title('Tractive Force — BMW i8 Hybrid (ICE rear + EM front)', ...
      'FontWeight','bold', 'FontSize', 14)
xlabel('Velocity [km/h]', 'FontWeight','bold', 'FontSize', 12)
ylabel('Force [N]',       'FontWeight','bold', 'FontSize', 12)
% FIX: RPM axis removed — was plotted as [N] which is dimensionally wrong.
%      Gear coverage is visible from the start/end speed of each curve.
grid on
xlim([0 260])
ylim([0 22000])

fprintf('  ✓ Complete (ICE rear + EM front, traction-limited per axle)\n\n');

% =========================================================================
%% 5. ACCELERATION ANALYSIS
% =========================================================================
fprintf('5. Acceleration Analysis...\n');

% --- 5a. Per-gear acceleration curves (combined powertrain) --------------
a_gear = zeros(n_gear, n_rpm);

figure('Name','5. Acceleration Performance','NumberTitle','off')
hold on

for g = 1:n_gear
  phi_g   = 1.04 + 0.0025 * (i_g(g) * i_f)^2;   % Inertia factor (Mitschke)
  % Interpolate resistance (0% grade) at gear's speed points
  FR_int  = interp1(vT, FR(1,:), v_ice(g,:), 'pchip', 'extrap');
  % Net force = combined tractive - resistance
  F_net_g = F_combined(g,:) - FR_int;
  a_gear(g,:) = max(F_net_g, 0) / (phi_g * m);   % Clamp at 0 [m/s²]
  plot(v_ice(g,:), a_gear(g,:), 'LineWidth', 1.5, ...
       'DisplayName', ['Gear ' num2str(g)])
end

title('Acceleration per Gear — BMW i8 (Combined Powertrain)', ...
      'FontWeight','bold', 'FontSize', 14)
xlabel('Velocity [km/h]', 'FontWeight','bold', 'FontSize', 12)
ylabel('Acceleration [m/s²]', 'FontWeight','bold', 'FontSize', 12)
legend('show', 'Location','northeast')
grid on

% =========================================================================
%% 5b. 0–100 km/h NUMERICAL INTEGRATION (was completely absent in original)
% =========================================================================
fprintf('\n  Running 0-100 km/h simulation...\n');

dt      = 0.005;        % Time step [s]
v_100   = 100 / 3.6;   % Target speed [m/s]
v_now   = 0.2;          % Initial speed [m/s] (avoid /0 in EM calculation)
t_now   = 0.0;
d_now   = 0.0;          % Distance [m]
v_log   = v_now;
t_log   = 0.0;
gear_log = 1;

while v_now < v_100 && t_now < 30

  % EM force at current speed
  omega_em_now = (v_now / Rd_f) * i_em;
  if omega_em_now <= omega_em_base
    T_em_now = T_em_peak;
  else
    T_em_now = P_em / omega_em_now;
  end
  F_em_now = eta_em * T_em_now * i_em / Rd_f;

  % Resistance (0% grade)
  fR_now    = interp1(vT_ref, fR_ref, max(v_now*3.6, 10), 'pchip', 'extrap');
  F_res_now = rho/2 * cd * A * v_now^2 + fR_now * G;

  % Find optimal gear (highest net acceleration)
  best_a   = -Inf;
  best_g   = 1;
  for g = 1:n_gear
    n_rpm_now = v_now / Rd * i_f * i_g(g) * 60 / (2*pi);
    if n_rpm_now < nm_ice(1) || n_rpm_now > nm_ice(end)
      continue   % Gear out of RPM range — skip
    end
    T_ice_now = interp1(nm_ice, Mm_ice, n_rpm_now, 'pchip');
    F_ice_now = eta_ice * T_ice_now * i_f * i_g(g) / Rd;
    phi_g     = 1.04 + 0.0025 * (i_g(g) * i_f)^2;

    % --- Traction limits with weight transfer (1-pass iteration) ---------
    %  First estimate acceleration without transfer:
    a_est       = (F_ice_now + F_em_now - F_res_now) / (phi_g * m);
    %  Update normal forces with weight transfer:
    N_rear_dyn  = G*sf/l + m * max(a_est,0) * h/l;
    N_front_dyn = max(G*sr/l - m * max(a_est,0) * h/l, 0);
    %  Apply traction caps:
    F_ice_lim   = min(F_ice_now, mu * N_rear_dyn);
    F_em_lim    = min(F_em_now,  mu * N_front_dyn);

    a_g = (F_ice_lim + F_em_lim - F_res_now) / (phi_g * m);

    if a_g > best_a
      best_a = a_g;
      best_g = g;
    end
  end

  if best_a <= 0 || isinf(best_a)
    break
  end

  % Euler integration
  v_now = v_now + best_a * dt;
  t_now = t_now + dt;
  d_now = d_now + v_now * dt;

  v_log(end+1)    = v_now;
  t_log(end+1)    = t_now;
  gear_log(end+1) = best_g;
end

% Interpolate exact crossing of 100 km/h
if v_now >= v_100
  t_0_100 = interp1(v_log(end-1:end), t_log(end-1:end), v_100);
  d_0_100 = interp1(t_log, d_now*ones(size(t_log)), t_0_100);
else
  t_0_100 = NaN;
end

% --- Report validation ---------------------------------------------------
fprintf('\n  ┌─────────────────────────────────────────┐\n');
fprintf('  │  0–100 km/h VALIDATION                  │\n');
fprintf('  ├─────────────────────────────────────────┤\n');
fprintf('  │  Simulated time : %5.2f s               │\n', t_0_100);
fprintf('  │  BMW published  :  4.40 s               │\n');
fprintf('  │  Error          : %+5.2f s  (%+.1f%%)        │\n', ...
        t_0_100 - 4.4, (t_0_100 - 4.4)/4.4 * 100);
fprintf('  └─────────────────────────────────────────┘\n');
fprintf('  Note: Error > 10%% → check i_em (EM reduction ratio).\n');
fprintf('  Tuning i_em changes v_em_base and peak EM tractive force.\n\n');

% --- Plot 0-100 velocity-time curve --------------------------------------
figure('Name','5b. 0-100 km/h Simulation','NumberTitle','off')
yyaxis left
  plot(t_log, v_log*3.6, 'b-', 'LineWidth', 2)
  yline(100, 'b--', '100 km/h')
  ylabel('Velocity [km/h]', 'FontWeight','bold')

yyaxis right
  stairs(t_log, gear_log, 'r-', 'LineWidth', 1.5)
  ylabel('Gear', 'FontWeight','bold')
  ylim([0.5 6.5])
  yticks(1:6)

xlabel('Time [s]', 'FontWeight','bold')
title(sprintf('0–100 km/h Simulation (t = %.2f s | BMW spec: 4.40 s)', t_0_100), ...
      'FontWeight','bold', 'FontSize', 13)
grid on
xlim([0 max(t_log)])

fprintf('  ✓ Acceleration analysis complete\n\n');

% =========================================================================
fprintf('========================================\n');
fprintf(' ALL ANALYSES COMPLETE\n');
fprintf('========================================\n');
fprintf('\n Generated Figures:\n');
fprintf('  1. Aerodynamic Drag\n');
fprintf('  2. Rolling Resistance\n');
fprintf('  3. Grade Resistance\n');
fprintf('  4. Tractive Force (Hybrid — ICE + EM)\n');
fprintf('  5. Acceleration per Gear\n');
fprintf('  5b. 0-100 km/h Time-Velocity Simulation\n\n');
fprintf(' Key model assumptions to validate/refine:\n');
fprintf('  - i_em = %.1f  (EM reduction ratio — most uncertain parameter)\n', i_em);
fprintf('  - mu  = %.2f  (friction coefficient — dry asphalt)\n', mu);
fprintf('  - h   = %.4f m (CoG height — statistically estimated)\n', h);
