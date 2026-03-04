% =========================================================================
% BMW i8 VEHICLE PERFORMANCE ANALYSIS - MAIN SCRIPT
% =========================================================================
%
% DESCRIPTION:
% This script performs comprehensive vehicle dynamics analysis for the 
% BMW i8 Coupe, including tractive force calculations, acceleration 
% performance, resistance forces, and power requirements across various 
% operating conditions.
%
% ANALYSIS COMPONENTS:
% 1. Aerodynamic Drag Analysis
%    - Calculates drag force and power requirements as function of velocity
%    - Uses vehicle-specific drag coefficient (0.26) and frontal area
%
% 2. Rolling Resistance Analysis  
%    - Computes tire rolling resistance for front and rear axles
%    - Accounts for tire pressure, load distribution, and velocity effects
%
% 3. Grade Resistance Analysis
%    - Analyzes power requirements for climbing various road gradients
%    - Generates 3D surface plot showing gradient vs velocity vs power
%
% 4. Tractive Force Calculation
%    - Calculates tractive force for all 6 gears across engine RPM range
%    - Plots total resistance for different road gradients (0% to 100%)
%    - Shows engine RPM curves and CVT comparison
%    - Determines maximum velocity and climbing capability for each gear
%
% 5. Acceleration Performance
%    - Computes acceleration for each gear considering inertia effects
%    - Accounts for resistance forces (aerodynamic, rolling, grade)
%    - Shows acceleration curves as function of velocity
%
% VEHICLE SPECIFICATIONS:
% - Vehicle: BMW i8 Coupe
% - Mass: 1530 kg
% - Power: 275 kW (max)
% - Gears: 6-speed (ratios: 4.46, 2.51, 1.56, 1.14, 0.85, 0.67)
% - Cd: 0.26, Frontal Area: 2.13 m²
%
% USAGE:
% Simply run this script to execute all analyses sequentially.
% All required vehicle data is loaded from VehicleData.m
% Each analysis generates its own figure window with results.
%
% OUTPUT:
% - Multiple figure windows showing various performance characteristics
% - All calculations stored in workspace variables for further analysis
%
% AUTHOR: [Your Name]
% DATE: January 2026
% =========================================================================

clear all
close all
clc

fprintf('========================================\n');
fprintf('BMW i8 VEHICLE PERFORMANCE ANALYSIS\n');
fprintf('========================================\n\n');

%% LOAD VEHICLE DATA
fprintf('Loading vehicle data...\n');
run('VehicleData')
fprintf('  ✓ Vehicle data loaded\n\n');

%% 1. AERODYNAMIC DRAG ANALYSIS
fprintf('Running Aerodynamic Drag Analysis...\n');

% Calculate air drag -------------------------
Fd = rho / 2 * cd * A * (vT/3.6).^2; % Aerodynamic drag [N]
Pd = Fd .* (vT/3.6); % Power [W]

% Graphs -------------------------------------
figure('Name','1. Aerodynamic Drag Analysis','NumberTitle','off')
subplot(1,2,1)
plot(vT,Fd,'LineWidth',2)
title('Aerodynamic drag vs. velocity','fontweight','bold')
xlabel('Velocity [km/h]','fontweight','bold')
ylabel('Force [N]','fontweight','bold')
grid on

subplot(1,2,2)
plot(vT,Pd/1000,'LineWidth',2)
title('Power vs. velocity','fontweight','bold')
xlabel('Velocity [km/h]','fontweight','bold')
ylabel('Power [kW]','fontweight','bold')
grid on

fprintf('  ✓ Aerodynamic drag analysis complete\n\n');

%% 2. ROLLING RESISTANCE ANALYSIS
fprintf('Running Rolling Resistance Analysis...\n');

% Rolling resistance calculation -----------------------------
Gtf = m * 9.81 * sr / l / 2; % Single wheel vertical load for front [N]
Gtr = m * 9.81 * sf / l / 2; % Single wheel vertical load for rear [N]

frf = fR * (1.3-0.3*pf/pTf)*(1.3-0.3*NTf/Gtf); % "Real" rolling resistance coefficient
frr = fR * (1.3-0.3*pr/pTr)*(1.3-0.3*NTr/Gtr); % "Real" rolling resistance coefficient

Ftf = frf * Gtf * 2; % Front axle rolling resistance [N]
Ftr = frr * Gtr * 2; % Rear axle rolling resistance [N]
Ft = Ftf + Ftr; % Rolling resistance for the whole vehicle [N]

Pt = vT/3.6 .* Ft; % Power calculated from rolling resistance [W]

% Graphs ------------------------------------------------------
figure('Name','2. Rolling Resistance Analysis','NumberTitle','off')

subplot(1,2,1)
plot(vT,Ft,'LineWidth',2)
title('Rolling resistance force vs. vehicle speed','fontweight','bold')
xlabel('Speed [km/h]','fontweight','bold')
ylabel('Force [N]','fontweight','bold')
grid on

subplot(1,2,2)
plot(vT,Pt,'LineWidth',2)
title('Power vs. vehicle speed','fontweight','bold')
xlabel('Speed [km/h]','fontweight','bold')
ylabel('Power [W]','fontweight','bold')
grid on

fprintf('  ✓ Rolling resistance analysis complete\n\n');

%% 3. GRADE RESISTANCE ANALYSIS
fprintf('Running Grade Resistance Analysis...\n');

Fg_grade = zeros(1,length(Grad)); % Creates a vector filled with zeros
Pg = zeros(length(Grad),length(vT));

% Grade resistance and power needed ---------------------
for i=1:length(Grad)
  alpha = atand(Grad(i)/100); % Calculate incline in degrees
  Fg_grade(i) = G * sind(alpha); % Grade resistance
  Pg(i,:) = vT / 3.6 * Fg_grade(i); % Power needed for the grade resistance and velocity range
end

% Graphs-----------------------------
figure('Name','3. Grade Resistance Analysis','NumberTitle','off')
surf(vT,Grad,Pg/1000)
title('Power needed for driving on an incline','fontweight','bold')
xlabel('Velocity [km/h]','fontweight','bold')
ylabel('Incline [%]','fontweight','bold')
zlabel('Power [kW]','fontweight','bold')
colorbar

fprintf('  ✓ Grade resistance analysis complete\n\n');

%% 4. TRACTIVE FORCE CALCULATION
fprintf('Running Tractive Force Calculation...\n');

% Create Variables / Vectors for calculation -----------------------
n = length(Grad); % Number of road inclines
nn = length(vT); % Length of the velocity vector
nnn = length(i_g); % Number of gears
Fg = zeros(n,1); % Grade resistance [N]
Gtf_trac = zeros(n,1); % Front tire load [N]
Gtr_trac = zeros(n,1); % Rear tire load [N]
frf_trac = zeros(n,nn); % Front tire rolling resistance coefficient
frr_trac = zeros(n,nn); % Rear tire rolling resistance coefficient
Fr = zeros(n,nn); % Total rolling resistance [N]
FR = zeros(n,nn); % Total driving resistance [N]

% Calculate air drag -----------------------------------------------
Fd_trac = rho / 2 * cd * A * (vT/3.6).^2; % Aerodynamic drag [N]

% Grade / Rolling / Total resistance -------------------------------
for i = 1:n
  alpha = atand(Grad(i)/100); % Calculate incline in degrees
  
  % Grade resistance -----------------------------------------------
  Fg(i) = G * sind(alpha); % Grade resistance [N]

  % Rolling resistance ---------------------------------------------
  Gtf_trac(i) = G * ( sr/l * cosd(alpha) - h/l * sind(alpha)) ; % Front axle tire load [N]
  Gtr_trac(i) = G * ( sf/l * cosd(alpha) + h/l * sind(alpha)) ; % Rear axle tire load [N]

  frf_trac(i,:) = fR * (1.3-0.3*pf/pTf)*(1.3-0.3*(NTf*2)/Gtf_trac(i)); % Real rolling resistance coefficient for front axle
  frr_trac(i,:) = fR * (1.3-0.3*pr/pTr)*(1.3-0.3*(NTr*2)/Gtr_trac(i)); % Real rolling resistance coefficient for rear axle

  Fr(i,:) = frf_trac(i,:)*Gtf_trac(i)+frr_trac(i,:)*Gtr_trac(i); % Total rolling resistance [N]

  % Total resistance [N]
  FR(i,:) = Fd_trac + Fg(i) + Fr(i,:); % Total resistance [N]
end

% Tractive force calculation----------------------------------------
v = zeros(nnn,length(nm)); % Tire tread velocity
Fx = zeros(nnn,length(nm)); % Tractive force [N]

for i=1:nnn
  v(i,:) = 2*pi*Rd*(nm/60) / (i_f*i_g(i))*3.6; % Velocity [km/h]
  Fx(i,:) = eta * Mm * i_f * i_g(i) / Rd; % Tractive force [N]
end

% Graphs -----------------------------------------------------------
figure('Name','4. Tractive Force Graph','NumberTitle','off')
hold on

% Resistance plots
for i = 1:n
  plot(vT,FR(i,:),'k','LineWidth',1)
end

% Text for the incline percentage
for i = 1:n
  grade = [' ' num2str(Grad(i)) '%'];
  text(vT(end),FR(i,end),grade);
end

% Tractive forces
for i = 1:nnn
  plot(v(i,:),Fx(i,:),'b','LineWidth',1)
end

% RPM graphs -------------------------------------------------------
for i= 1:nnn
  plot(v(i,:),nm,'-.r','LineWidth',1)
end

% Add gear numbers -------------------------------------------------
for i = 1:nnn
  gear = [' ' num2str(i) '.'];
  text(v(i,end),nm(end)+20,gear);
end

% CVT-graph ---------------------------------------------------------
Fcvt = Pmax * eta ./ (vT/3.6); % Tractive force for CVT-gear box
plot(vT,Fcvt,'--k','LineWidth',1)

title('Tractive force graph','fontweight','bold','fontsize',18)
xlabel('Velocity [km/h]','fontweight','bold','fontsize',14)
ylabel('Force [N] / Resistance [N] / RPM','fontweight','bold','fontsize',14)
grid on
xlim([0 300])
ylim([0 30000])

fprintf('  ✓ Tractive force calculation complete\n\n');

%% 5. ACCELERATION ANALYSIS
fprintf('Running Acceleration Analysis...\n');

n_gears = length(i_g); % Measure the number of gears
a = zeros(n_gears,length(Mm)); % Create an empty matrice for acceleration

figure('Name','5. Acceleration Performance','NumberTitle','off')
hold on

% Calculate acceleration for each gear
for i=1:n_gears
  phi = 1.04+0.0025 * (i_g(i)*i_f)^2; % Inertia factor
  FR_Int = interp1(vT,FR(1,:),v(i,:),'pchip','extrap'); % Interpolated resistance on 0% incline
  a(i,:) = (Fx(i,:)-FR_Int) / (phi*m);

  plot(v(i,:),a(i,:),'LineWidth',1.5)
end

title('Acceleration for each gear','fontweight','bold','fontsize',18)
xlabel('Velocity [km/h]','fontweight','bold','fontsize',14)
ylabel('Acceleration [m/s^2]','fontweight','bold','fontsize',14)
grid on

fprintf('  ✓ Acceleration analysis complete\n\n');

%% ANALYSIS COMPLETE
fprintf('========================================\n');
fprintf('ALL ANALYSES COMPLETED SUCCESSFULLY!\n');
fprintf('========================================\n');
fprintf('\nGenerated Figures:\n');
fprintf('  1. Aerodynamic Drag Analysis\n');
fprintf('  2. Rolling Resistance Analysis\n');
fprintf('  3. Grade Resistance Analysis\n');
fprintf('  4. Tractive Force Graph\n');
fprintf('  5. Acceleration Performance\n\n');
