%% =========================================================================
%  SURA AIRSHIP — COMPLETE SIZING & ENERGY SIMULATION SCRIPT
%  Agya Sanghi & Kartik Aggarwal | IIT Delhi | SURA 2025
%
%  Airship Type   : Autonomous Solar-Assisted LTA Delivery Airship
%  Hull Shape     : Prolate Spheroid (fineness ratio k = 3.5)
%  Payload        : 2 kg, winch-deployed (Yo-Yo manoeuvre)
%  Mission        : 12-hour net-zero diurnal cycle
%
%  Actuator Configuration (7 total):
%    Motor 1 : T-Motor MN3110 KV470 — Port rear cruise
%    Motor 2 : T-Motor MN3110 KV470 — Starboard rear cruise
%    Motor 3 : T-Motor MN3110 KV470 — Vertical (RL buoyancy shock control)
%    Motor 4 : Pololu 37D 19:1 Gear Motor — Winch (payload deploy/retrieve)
%    Servo 1 : Hitec HS-65HB — Elevator (pitch control)
%    Servo 2 : Hitec HS-65HB — Rudder (yaw control)
%    Servo 3 : Hitec HS-65HB — Payload latch release
%
%  All parameters sourced from manufacturer datasheets.
%  All efficiencies use industry-standard conservative values.
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf(' SURA AIRSHIP — PHYSICS-BASED SIZING & ENERGY SIMULATION\n');
fprintf(' IIT Delhi | SURA 2025\n');
fprintf('=================================================================\n\n');

%% =========================================================================
%  SECTION 1: PHYSICAL CONSTANTS
% =========================================================================

g        = 9.81;          % [m/s²]   gravitational acceleration
rho_air  = 1.225;         % [kg/m³]  air density at sea level, ISA (15°C)
rho_He   = 0.1664;        % [kg/m³]  helium density at STP (NIST)
% Note: rho_air - rho_He = 1.0586 kg/m³ → net buoyant lift per m³

fprintf('[1] Physical Constants\n');
fprintf('    Air density      : %.4f kg/m³\n', rho_air);
fprintf('    Helium density   : %.4f kg/m³\n', rho_He);
fprintf('    Net lift/m³      : %.4f kg/m³\n', (rho_air - rho_He));
fprintf('\n');

%% =========================================================================
%  SECTION 2: HULL GEOMETRY — PROLATE SPHEROID
%  Shape: semi-major axis a (length), semi-minor axis b (radius)
%  Fineness ratio k = 2a / 2b = a/b
%  Optimal drag: k = 3.5 (Lutz & Munson, AIAA — minimum CDv in LTA range)
% =========================================================================

k        = 3.5;           % [-]      fineness ratio (length/diameter)
% k=3.5 gives CDv=0.030 (published, AIAA blimp aerodynamics)
% k=4.0 gives CDv=0.028 — marginal improvement, k=3.5 is standard

CDv      = 0.030;         % [-]      volumetric drag coefficient (Lutz & Munson)
% Verified range: CDv = 0.025–0.035 for k = 3–5 prolate spheroids

% Hull geometry derived from volume V (solved later via fzero)
% a = semi-major (half-length), b = semi-minor (half-radius)
% V = (4/3)*pi*a*b^2
% k = a/b  →  a = k*b
% V = (4/3)*pi*(k*b)*b^2 = (4/3)*pi*k*b^3
% b = (3V / (4*pi*k))^(1/3)

fprintf('[2] Hull Geometry\n');
fprintf('    Shape            : Prolate Spheroid\n');
fprintf('    Fineness ratio k : %.1f\n', k);
fprintf('    CDv              : %.3f\n', CDv);
fprintf('\n');

%% =========================================================================
%  SECTION 3: ENVELOPE MATERIAL
%  Material: Heavy-duty PET/Mylar laminate (UV-resistant, outdoor-rated)
%  Industry standard for advertising blimps and RC LTA vehicles
%  Source: Lindstrand Technologies, Aerostar International datasheets
% =========================================================================

sigma_env = 0.070;        % [kg/m²]  envelope areal density
% Range in literature:
%   Ultra-light TPU ripstop nylon (indoor RC): 0.035–0.050 kg/m²
%   PET/Mylar laminate (outdoor, UV-rated)  : 0.060–0.075 kg/m²  ← used
%   Professional Lindstrand/Aerostar        : 0.070–0.090 kg/m²
% 0.070 kg/m² is conservative-realistic for a student outdoor prototype.

% Prolate spheroid surface area formula (exact, arc-sin eccentricity form)
% S = 2*pi*b^2 * (1 + (a/b) * asin(e)/e)   where e = sqrt(1-(b/a)^2)
% This is evaluated inside fzero loop below.

fprintf('[3] Envelope Material\n');
fprintf('    Material         : PET/Mylar laminate (UV-resistant)\n');
fprintf('    sigma_env        : %.3f kg/m²\n', sigma_env);
fprintf('\n');

%% =========================================================================
%  SECTION 4: HARDWARE MASS BUDGET (BOM-VERIFIED)
%  All masses from manufacturer datasheets. See component analysis doc.
% =========================================================================

% --- PROPULSION ---
m_motor_MN3110   = 0.099;   % [kg]  T-Motor MN3110 KV470 (store.tmotor.com)
                             %       Includes motor body + cables (80g+19g)
m_motor_count    = 3;        % 3x MN3110: port, starboard, vertical
m_motors_cruise  = m_motor_MN3110 * m_motor_count;  % 0.297 kg

m_motor_winch    = 0.160;   % [kg]  Pololu 37D 19:1 Gear Motor (pololu.com)
                             %       160g, 12V, 300RPM, ~6W rated

m_ESC_cruise     = 0.028;   % [kg]  T-Motor AM 30A BLHeli ESC (store.tmotor.com)
m_ESC_count      = 3;        % one ESC per MN3110
m_ESCs_cruise    = m_ESC_cruise * m_ESC_count;      % 0.084 kg

m_ESC_winch      = 0.030;   % [kg]  Cytron 13A DC Motor Driver (cytron.io)
                             %       Bidirectional, 6–30V, 30g

m_prop           = 0.032;   % [kg]  T-Motor 15×5 inch CF pair (store.tmotor.com)
m_prop_count     = 3;        % all 3 MN3110 motors carry props
m_props          = m_prop * m_prop_count;            % 0.096 kg

m_propulsion = m_motors_cruise + m_motor_winch + m_ESCs_cruise + ...
               m_ESC_winch + m_props;

% --- CONTROL SURFACES & SERVOS ---
m_servo_HS65HB   = 0.009;   % [kg]  Hitec HS-65HB (hitecrcd.com) — 9.1g
m_servo_count    = 3;        % elevator + rudder + payload latch
m_servos         = m_servo_HS65HB * m_servo_count;  % 0.027 kg

m_tail_fins      = 0.040;   % [kg]  4x tail fins: 3mm CF rod + EPP foam
                             %       ~10g per fin × 4 fins = 40g total

m_control = m_servos + m_tail_fins;

% --- FLIGHT COMPUTERS ---
m_pixhawk_6C     = 0.032;   % [kg]  Holybro Pixhawk 6C Mini (holybro.com)
                             %       STM32H743, ArduBlimp compatible, 32g with case
m_rpi4           = 0.046;   % [kg]  Raspberry Pi 4B 4GB (raspberrypi.com)
                             %       46g; runs YOLOv8 + ROS2 Nav2
m_rpi_ai_cam     = 0.014;   % [kg]  RPi AI Camera IMX500 (raspberrypi.com)
                             %       13MP, on-sensor YOLOv8 inference, 14g

m_computers = m_pixhawk_6C + m_rpi4 + m_rpi_ai_cam;

% --- NAVIGATION & SENSING ---
m_GPS_M10        = 0.036;   % [kg]  Holybro M10 GPS + IST8310 Magnetometer
                             %       36g including mast; uBlox M10 chip
m_telemetry      = 0.036;   % [kg]  Holybro SiK Telemetry v3 433MHz (holybro.com)
                             %       1W output, 300m+ range, 36g
% Barometric altimeter: MS5611 integrated on Pixhawk — no extra mass

m_navigation = m_GPS_M10 + m_telemetry;

% --- POWER SYSTEM ---
m_battery        = 0.620;   % [kg]  Tattu 4S 10000mAh 15C LiPo (genstattu.com)
                             %       620g, 14.8V, 148 Wh rated capacity
m_solar_module   = 0.082;   % [kg]  Ascent Solar HL-25 HyperLight CIGS 25W module
                             %       (pv-magazine.com) — 82g per 25W module
m_solar_count    = 8;        % 8 modules × 25W = 200W peak array
m_solar_array    = m_solar_module * m_solar_count;  % 0.656 kg

m_MPPT           = 0.057;   % [kg]  Genasun GVB-8 LiPo MPPT (genasun.com)
                             %       8A, 5–35V input, designed for LTA/UAV, 57g
m_PDB            = 0.014;   % [kg]  Matek HUBOSD8 PDB (mateksys.com)
                             %       14g, integrated 5V/12V BECs

m_power = m_battery + m_solar_array + m_MPPT + m_PDB;

% --- PAYLOAD MECHANISM ---
m_winch_rope     = 0.035;   % [kg]  20m Dyneema SK75 1mm cord + spool
                             %       1.6 g/m × 20m = 32g + 3g spool ≈ 35g
m_latch          = 0.000;   % already included in m_servos above
m_carabiner      = 0.014;   % [kg]  Aluminium 30mm locking carabiner — 14g

m_payload_mech = m_winch_rope + m_carabiner;

% --- STRUCTURAL & MISC ---
m_wiring         = 0.080;   % [kg]  XT60 plugs, silicone wire, JST connectors
m_gondola        = 0.120;   % [kg]  3D-printed PLA + CF tube gondola frame
m_dampers        = 0.010;   % [kg]  Silicone vibration standoffs ×8 (Pixhawk isolation)

m_structural = m_wiring + m_gondola + m_dampers;

% --- TOTAL HARDWARE MASS ---
M_hw = m_propulsion + m_control + m_computers + m_navigation + ...
       m_power + m_payload_mech + m_structural;

fprintf('[4] Hardware Mass Budget (BOM-Verified)\n');
fprintf('    Propulsion       : %.3f kg\n', m_propulsion);
fprintf('      └ 3x MN3110 motors    : %.3f kg\n', m_motors_cruise);
fprintf('      └ Winch gear motor    : %.3f kg\n', m_motor_winch);
fprintf('      └ 3x cruise ESCs      : %.3f kg\n', m_ESCs_cruise);
fprintf('      └ Winch motor driver  : %.3f kg\n', m_ESC_winch);
fprintf('      └ 3x CF props 15x5    : %.3f kg\n', m_props);
fprintf('    Control Surfaces : %.3f kg\n', m_control);
fprintf('      └ 3x Hitec HS-65HB   : %.3f kg\n', m_servos);
fprintf('      └ 4x tail fins       : %.3f kg\n', m_tail_fins);
fprintf('    Flight Computers : %.3f kg\n', m_computers);
fprintf('      └ Pixhawk 6C Mini    : %.3f kg\n', m_pixhawk_6C);
fprintf('      └ Raspberry Pi 4B    : %.3f kg\n', m_rpi4);
fprintf('      └ RPi AI Camera      : %.3f kg\n', m_rpi_ai_cam);
fprintf('    Navigation       : %.3f kg\n', m_navigation);
fprintf('    Power System     : %.3f kg\n', m_power);
fprintf('      └ Tattu 4S 10Ah LiPo : %.3f kg\n', m_battery);
fprintf('      └ 8x Ascent HL-25    : %.3f kg\n', m_solar_array);
fprintf('      └ Genasun GVB-8 MPPT : %.3f kg\n', m_MPPT);
fprintf('      └ Matek PDB          : %.3f kg\n', m_PDB);
fprintf('    Payload Mechanism: %.3f kg\n', m_payload_mech);
fprintf('    Structural/Misc  : %.3f kg\n', m_structural);
fprintf('    ─────────────────────────────────\n');
fprintf('    TOTAL M_hw       : %.3f kg\n\n', M_hw);

%% =========================================================================
%  SECTION 5: PAYLOAD
% =========================================================================

m_payload = 2.000;        % [kg]  design payload (maximum delivery mass)

fprintf('[5] Payload\n');
fprintf('    m_payload        : %.3f kg\n\n', m_payload);

%% =========================================================================
%  SECTION 6: MOTOR & DRIVETRAIN EFFICIENCIES (Industry Standards)
%
%  T-Motor MN3110 KV470 (brushless outrunner):
%    Motor electrical efficiency: 82–85% at cruise load
%    Source: T-Motor MN3110 performance table, 4S 14.8V
%    Conservative value: 82% (partial-load operation)
%
%  T-Motor 15×5 CF Propeller:
%    Propeller aerodynamic efficiency at optimal advance ratio: 78–82%
%    At LTA cruise (low speed, low RPM): 78% conservative
%    Source: UIUC Prop Database, large-diameter low-pitch props
%
%  Combined drivetrain (motor × prop): 0.82 × 0.78 = 0.640
%
%  Pololu 37D 19:1 Gear Motor:
%    Motor efficiency: 70% (DC brush motor at rated load)
%    Gearbox efficiency: 85% (19:1 metal spur gearbox, Pololu spec)
%    Combined: 0.70 × 0.85 = 0.595 ≈ 60%
%    Source: Pololu 37D series datasheet
%
%  Hitec HS-65HB Servo:
%    Input power: 4.8V, stall torque 1.5 kg·cm
%    Average power at partial duty: ~0.5–0.7W per servo
%    3 servos total average power: ~1.8W
%
%  Genasun GVB-8 MPPT:
%    MPPT tracking efficiency: 99.2% (Genasun spec — synchronous buck)
%    Power conversion efficiency: 96% (buck converter at rated load)
%    End-to-end solar-to-battery: 0.992 × 0.960 = 0.952 ≈ 95%
%    Source: Genasun GVB-8 datasheet, genasun.com
%
%  Tattu 4S LiPo Battery:
%    Discharge efficiency: 98% at 1C rate (standard LiPo)
%    Depth of discharge limit: 80% (never below 20% SoC — cell health)
%    Usable capacity: 148 Wh × 0.80 = 118.4 Wh
%
%  Ascent Solar HL-25 CIGS thin-film:
%    Rated module efficiency: 11% (Ascent Solar spec, pv-magazine 2021)
%    Field derating (heat, partial cloud, India summer): ×0.88
%    Effective field efficiency: 11% × 0.88 = 9.68% → use 10%
%    With MPPT efficiency (95%): effective system = 10% × 0.95 = 9.5%
%    Conservative simulation value: eta_solar = 0.10
%    Note: even at 10%, 2m² × 1000 W/m² × 0.10 = 200W peak → net-positive
% =========================================================================

eta_motor        = 0.82;   % [-]  T-Motor MN3110 electrical efficiency (datasheet)
eta_prop         = 0.78;   % [-]  T-Motor 15×5 CF prop aerodynamic efficiency
eta_drivetrain   = eta_motor * eta_prop;  % 0.6396 → 64%

eta_winch_motor  = 0.70;   % [-]  DC brush motor efficiency (Pololu 37D)
eta_winch_gear   = 0.85;   % [-]  19:1 metal gearbox efficiency (Pololu spec)
eta_winch        = eta_winch_motor * eta_winch_gear;  % 0.595

eta_solar        = 0.10;   % [-]  CIGS effective field efficiency (conservative)
                            %      Ascent HL-25 rated 11%, field-derated to 10%
eta_MPPT         = 0.952;  % [-]  Genasun GVB-8 end-to-end (0.992 × 0.960)
eta_battery_dis  = 0.98;   % [-]  LiPo discharge efficiency at 1C
DoD_max          = 0.80;   % [-]  maximum depth of discharge (80% — LiPo standard)

fprintf('[6] Component Efficiencies (Industry Standard Values)\n');
fprintf('    Motor (MN3110 electrical)  : %.2f (%.0f%%)\n', eta_motor, eta_motor*100);
fprintf('    Propeller (15x5 CF)        : %.2f (%.0f%%)\n', eta_prop, eta_prop*100);
fprintf('    Combined drivetrain        : %.4f (%.1f%%)\n', eta_drivetrain, eta_drivetrain*100);
fprintf('    Winch motor (DC brush)     : %.2f (%.0f%%)\n', eta_winch_motor, eta_winch_motor*100);
fprintf('    Winch gearbox (19:1)       : %.2f (%.0f%%)\n', eta_winch_gear, eta_winch_gear*100);
fprintf('    Winch combined             : %.4f (%.1f%%)\n', eta_winch, eta_winch*100);
fprintf('    Solar CIGS (field eff.)    : %.2f (%.0f%%)\n', eta_solar, eta_solar*100);
fprintf('    MPPT (Genasun GVB-8)       : %.4f (%.2f%%)\n', eta_MPPT, eta_MPPT*100);
fprintf('    Battery discharge          : %.2f (%.0f%%)\n', eta_battery_dis, eta_battery_dis*100);
fprintf('    Battery max DoD            : %.0f%%\n\n', DoD_max*100);

%% =========================================================================
%  SECTION 7: CRUISE PERFORMANCE & DRAG
%  Volumetric drag equation for LTA vehicles (AIAA standard form):
%    D = 0.5 * rho_air * v^2 * V^(2/3) * CDv
%  Units: [N] = [kg/m³] × [m/s]² × [m²] × [-]
% =========================================================================

v_cruise = 5.0;           % [m/s]  design cruise speed
                           %        Realistic for indoor/campus airship
                           %        Wind resistance target: 5 m/s headwind

% Drag will be evaluated at the solved volume — placeholder here
% Final drag computed after fzero solves for V

fprintf('[7] Cruise Performance\n');
fprintf('    Design cruise speed : %.1f m/s\n\n', v_cruise);

%% =========================================================================
%  SECTION 8: POWER BUDGET
%
%  Propulsion power derivation:
%    Required thrust F_thrust = Drag D at cruise speed
%    Shaft power = F_thrust × v_cruise / eta_prop
%    Electrical power = Shaft power / eta_motor
%    Per motor: P_motor_elec = (D × v_cruise) / (2 × eta_drivetrain)
%    (factor 2: two rear cruise motors share the thrust equally)
%
%  Vertical motor (RL buoyancy shock):
%    Active only during shock arrest (~30s per delivery, 4 deliveries)
%    Peak power: ~40W electrical
%    Average over mission: negligible contribution
%
%  Winch motor:
%    Mechanical power to lift/lower: F × v_rope
%    F = m_payload × g = 2.0 × 9.81 = 19.62 N
%    v_rope ≈ 0.1 m/s (controlled descent, 10m in ~100s)
%    Mechanical power = 19.62 × 0.1 = 1.962 W
%    Electrical power = 1.962 / eta_winch = 1.962 / 0.595 = 3.3 W
%    Add idle/holding current: total ~6W (conservative, matches Pololu spec)
%
%  Avionics power:
%    Pixhawk 6C Mini         : 2.5 W  (STM32H743, sensors active)
%    Raspberry Pi 4B (YOLOv8): 7.0 W  (inference at ~60% CPU load)
%    RPi AI Camera IMX500    : 1.5 W  (on-sensor inference reduces RPi load)
%    Holybro M10 GPS         : 0.5 W  (uBlox M10, active tracking)
%    SiK Telemetry v3        : 1.0 W  (433MHz, 1W TX, ~30% duty cycle)
%    3× Hitec HS-65HB Servos : 1.8 W  (partial duty, ~0.6W each average)
%    Misc (LEDs, logic)      : 0.7 W
%    Total avionics          : 15.0 W
% =========================================================================

% These are used inside fzero — computed after volume is known
% Avionics baseline (independent of volume)
P_avionics = 15.0;        % [W]  total avionics steady-state power draw

% Winch motor power
P_winch = 6.0;             % [W]  electrical power during winch operation
                            %      Mechanical: ~3.3W / eta_winch = 5.5W → 6W

% Servo power (included in P_avionics above)
% Vertical motor: only during shock events — computed separately in Section 10

fprintf('[8] Power Budget (Fixed Components)\n');
fprintf('    P_avionics (Pixhawk+RPi4+camera+GPS+telemetry+servos): %.1f W\n', P_avionics);
fprintf('    P_winch (Pololu 37D, payload operation)               : %.1f W\n', P_winch);
fprintf('\n');

%% =========================================================================
%  SECTION 9: SOLAR ARRAY
%  8× Ascent Solar HL-25 HyperLight CIGS 25W modules
%  Total rated: 200W at AM1.5, 1000 W/m² reference
%  Physical area: each module ~0.25 m² (25W / (1000 W/m² × 0.10))
%  Total array area: 8 × 0.25 = 2.0 m²
%
%  Irradiance model: sinusoidal diurnal profile
%    G(t) = G_peak × sin(π × (t-t_rise) / t_day)  for t_rise ≤ t ≤ t_set
%    G(t) = 0                                       otherwise
%  Solar window: 06:00–18:00 (12-hour day, conservative for India)
%  Peak irradiance: 1000 W/m² (AM1.5 standard, valid for N. India summer)
%
%  Solar power:
%    P_solar(t) = eta_solar × eta_MPPT × A_solar × G(t)
% =========================================================================

A_solar     = 2.0;         % [m²]   total panel area (8× HL-25 modules)
G_peak      = 1000.0;      % [W/m²] peak solar irradiance (AM1.5)
t_rise      = 6.0;         % [h]    sunrise (06:00)
t_set       = 18.0;        % [h]    sunset (18:00)
t_day       = t_set - t_rise;  % [h] = 12 hours

% Time vector: full 24-hour mission at 1-minute resolution
dt          = 1/60;        % [h]    time step = 1 minute
t_vec       = 0:dt:24;     % [h]    time array 0–24h

% Irradiance profile
G_t = zeros(size(t_vec));
solar_mask = (t_vec >= t_rise) & (t_vec <= t_set);
G_t(solar_mask) = G_peak .* sin(pi .* (t_vec(solar_mask) - t_rise) ./ t_day);

% Solar power harvested
P_solar_t = eta_solar .* eta_MPPT .* A_solar .* G_t;  % [W]

% Total solar energy over 12h (integrate)
E_solar_total = trapz(t_vec, P_solar_t);               % [Wh]

% Analytical verification: integral of sin over [0,T] = 2T/π
E_solar_analytical = eta_solar * eta_MPPT * A_solar * G_peak * t_day * 2/pi;

fprintf('[9] Solar Array\n');
fprintf('    Array             : 8× Ascent Solar HL-25 CIGS 25W\n');
fprintf('    Total rated power : %.0f W\n', 8*25);
fprintf('    Array area        : %.1f m²\n', A_solar);
fprintf('    Field efficiency  : %.0f%% (CIGS, derated for heat/cloud)\n', eta_solar*100);
fprintf('    MPPT efficiency   : %.1f%% (Genasun GVB-8)\n', eta_MPPT*100);
fprintf('    Peak P_solar      : %.1f W  (at noon)\n', eta_solar*eta_MPPT*A_solar*G_peak);
fprintf('    Total solar energy (numerical) : %.1f Wh\n', E_solar_total);
fprintf('    Total solar energy (analytical): %.1f Wh\n', E_solar_analytical);
fprintf('\n');

%% =========================================================================
%  SECTION 10: VOLUME SIZING VIA MASS BALANCE (fzero)
%
%  At equilibrium: Net lift = Total weight
%    (rho_air - rho_He) × V × g = m_total × g
%    m_total = m_envelope + M_hw + m_payload
%    m_envelope = sigma_env × S(V)
%
%  Prolate spheroid surface area (exact):
%    e = sqrt(1 - (b/a)²) = sqrt(1 - 1/k²)
%    S = 2π b² (1 + (a/b) × arcsin(e)/e)
%    where b = (3V/(4π k))^(1/3), a = k×b
%
%  Mass balance function:
%    f(V) = (rho_air - rho_He)×V - sigma_env×S(V) - M_hw - m_payload = 0
% =========================================================================

mass_balance = @(V) mass_balance_fn(V, rho_air, rho_He, sigma_env, k, M_hw, m_payload);

% Initial guess and bounds
V_guess = 10.0;            % [m³]  initial guess
V_min   = 1.0;             % [m³]  lower bound
V_max   = 100.0;           % [m³]  upper bound

options = optimset('TolX', 1e-8, 'Display', 'off');
[V_eq, ~, exitflag] = fzero(mass_balance, V_guess, options);

if exitflag ~= 1
    warning('fzero did not converge — check initial guess and bounds');
end

% Derive geometry from solved volume
b_eq = (3*V_eq / (4*pi*k))^(1/3);   % [m]  semi-minor axis (radius)
a_eq = k * b_eq;                      % [m]  semi-major axis (half-length)
D_eq = 2 * b_eq;                      % [m]  maximum diameter
L_eq = 2 * a_eq;                      % [m]  total length

% Surface area at equilibrium volume
e_eq   = sqrt(1 - (b_eq/a_eq)^2);
S_eq   = 2*pi*b_eq^2 * (1 + (a_eq/b_eq) * asin(e_eq)/e_eq);

% Mass breakdown at equilibrium
m_env_eq = sigma_env * S_eq;
m_total  = m_env_eq + M_hw + m_payload;
F_buoy   = (rho_air - rho_He) * V_eq * g;
F_weight = m_total * g;

fprintf('[10] Volume Sizing (fzero mass balance)\n');
fprintf('    ─────────────────────────────────\n');
fprintf('    Equilibrium volume    V : %.4f m³\n', V_eq);
fprintf('    Semi-minor axis       b : %.4f m   (radius at widest)\n', b_eq);
fprintf('    Semi-major axis       a : %.4f m   (half-length)\n', a_eq);
fprintf('    Maximum diameter      D : %.4f m\n', D_eq);
fprintf('    Total hull length     L : %.4f m\n', L_eq);
fprintf('    Surface area          S : %.4f m²\n', S_eq);
fprintf('    ─────────────────────────────────\n');
fprintf('    Envelope mass  m_env   : %.4f kg\n', m_env_eq);
fprintf('    Hardware mass  M_hw    : %.4f kg\n', M_hw);
fprintf('    Payload mass   m_pay   : %.4f kg\n', m_payload);
fprintf('    TOTAL mass     m_total : %.4f kg\n', m_total);
fprintf('    ─────────────────────────────────\n');
fprintf('    Buoyancy force         : %.4f N\n', F_buoy);
fprintf('    Weight force           : %.4f N\n', F_weight);
fprintf('    Balance error          : %.2e N  (should be ~0)\n\n', F_buoy - F_weight);

%% =========================================================================
%  SECTION 11: AERODYNAMIC DRAG & PROPULSION POWER
% =========================================================================

D_drag = 0.5 * rho_air * v_cruise^2 * V_eq^(2/3) * CDv;  % [N]

% Thrust power (mechanical, overcoming drag)
P_thrust_mech = D_drag * v_cruise;   % [W]  = Force × velocity

% Electrical power to 2 cruise motors (port + starboard share equally)
P_cruise_elec = P_thrust_mech / eta_drivetrain;  % [W]  total electrical

% Per-motor electrical power
P_per_motor = P_cruise_elec / 2;     % [W]  each rear motor

% Thrust safety margin check
% At full load (not cruise), MN3110 KV470 on 4S can deliver ~60W each
% Operating at P_per_motor/60W = headroom check
headroom_pct = (1 - P_per_motor/60) * 100;

fprintf('[11] Aerodynamic Drag & Propulsion\n');
fprintf('    Drag force at %.1f m/s    : %.4f N\n', v_cruise, D_drag);
fprintf('    Thrust mech. power       : %.4f W\n', P_thrust_mech);
fprintf('    Total electrical (cruise): %.4f W  (both motors)\n', P_cruise_elec);
fprintf('    Per rear motor           : %.4f W\n', P_per_motor);
fprintf('    Motor headroom           : %.1f%% (max 60W each on 4S)\n\n', headroom_pct);

%% =========================================================================
%  SECTION 12: BUOYANCY SHOCK ANALYSIS
%  Event: payload (2 kg) released → sudden buoyancy excess
%
%  Pre-drop: F_net = 0 (neutral buoyancy, in equilibrium)
%  Post-drop: F_net_up = m_payload × g = 2.0 × 9.81 = 19.62 N (upward)
%
%  Vertical motor (MN3110 KV470) downward thrust requirement:
%    Max static thrust of MN3110 on 4S with 15×5 prop:
%    From T-Motor thrust table: ~800g at 50% throttle, ~1600g at 100%
%    Required: 19.62 N = 2000g → needs ~65% throttle → feasible
%
%  Electrical power during buoyancy shock arrest:
%    At 65% throttle, MN3110 draws ~35–40W electrical
%    Use 40W (conservative peak for vertical motor)
%
%  Duration: 30 seconds per event, 4 deliveries per mission
%  Energy: 40W × (4 × 30s / 3600) = 40 × 0.0333 = 1.33 Wh
% =========================================================================

F_shock       = m_payload * g;        % [N]  buoyancy surplus post-drop = 19.62 N
P_vert_peak   = 40.0;                 % [W]  vertical motor electrical at ~65% throttle
t_shock_s     = 30.0;                 % [s]  shock arrest duration per event
n_deliveries  = 4;                    % [-]  deliveries per mission day
E_shock_total = P_vert_peak * (n_deliveries * t_shock_s / 3600);  % [Wh]

fprintf('[12] Buoyancy Shock Analysis\n');
fprintf('    Payload mass              : %.2f kg\n', m_payload);
fprintf('    Buoyancy surplus (post-drop): %.2f N  (%.0f gf)\n', F_shock, F_shock/g*1000);
fprintf('    Vertical motor peak power : %.1f W  (MN3110, ~65%% throttle)\n', P_vert_peak);
fprintf('    Shock duration per event  : %.0f s\n', t_shock_s);
fprintf('    Deliveries per mission    : %d\n', n_deliveries);
fprintf('    Total shock energy        : %.2f Wh\n\n', E_shock_total);

%% =========================================================================
%  SECTION 13: COMPLETE MISSION POWER & ENERGY BUDGET
%
%  Mission profile (12-hour day):
%    Transit cruise (5 m/s, 2 rear motors + avionics) : 8 hours
%    Payload deployment (hover, vert. motor station-keep + avionics): 2 hours
%    Buoyancy shock spikes (vert. motor burst): 4 × 30s = 0.033 hours
%    Winch operation (lower + raise, 4 deliveries): 4 × 5 min = 0.333 hours
%    Night (avionics standby only, solar off): 12 hours
%
%  Power modes:
%    Transit cruise   : P = P_cruise_elec + P_avionics
%    Hover/deploy     : P = P_vert_hover + P_avionics  (vert. motor ~8W)
%    Shock burst      : P = P_vert_peak + P_avionics   (vert. motor 40W)
%    Winch operation  : P = P_winch + P_avionics
%    Night standby    : P = P_standby (Pixhawk+GPS minimal = 4W)
% =========================================================================

% Power in each mode [W]
P_hover_vert  = 8.0;       % [W]  vertical motor during station-keep hover (low thrust)
P_standby     = 4.0;       % [W]  night standby (Pixhawk + GPS minimal mode)

P_mode_cruise  = P_cruise_elec + P_avionics;
P_mode_hover   = P_hover_vert  + P_avionics;
P_mode_shock   = P_vert_peak   + P_avionics;
P_mode_winch   = P_winch       + P_avionics;
P_mode_night   = P_standby;

% Duration in each mode [hours]
t_cruise_h     = 8.0;
t_hover_h      = 2.0;
t_shock_h      = n_deliveries * t_shock_s / 3600;   % 0.0333 h
t_winch_h      = n_deliveries * 5 / 60;             % 4×5min = 0.333 h
t_night_h      = 12.0;                               % 12 hours dark

% Energy in each mode [Wh]
E_cruise  = P_mode_cruise * t_cruise_h;
E_hover   = P_mode_hover  * t_hover_h;
E_shock   = P_mode_shock  * t_shock_h;
E_winch   = P_mode_winch  * t_winch_h;
E_night   = P_mode_night  * t_night_h;

E_total_consumed = E_cruise + E_hover + E_shock + E_winch + E_night;

fprintf('[13] Mission Power & Energy Budget\n');
fprintf('    %-30s %8s %8s %10s\n', 'Mode', 'Power(W)', 'Time(h)', 'Energy(Wh)');
fprintf('    %-30s %8.1f %8.1f %10.1f\n', 'Transit cruise (2 rear motors)', P_mode_cruise, t_cruise_h, E_cruise);
fprintf('    %-30s %8.1f %8.1f %10.1f\n', 'Payload hover (vert motor)', P_mode_hover, t_hover_h, E_hover);
fprintf('    %-30s %8.1f %8.3f %10.2f\n', 'Buoyancy shock burst (x4)', P_mode_shock, t_shock_h, E_shock);
fprintf('    %-30s %8.1f %8.3f %10.1f\n', 'Winch operation (x4, 5min)', P_mode_winch, t_winch_h, E_winch);
fprintf('    %-30s %8.1f %8.1f %10.1f\n', 'Night standby (12h)', P_mode_night, t_night_h, E_night);
fprintf('    ──────────────────────────────────────────────────────\n');
fprintf('    %-30s %8s %8s %10.1f\n', 'TOTAL CONSUMED', '', '', E_total_consumed);
fprintf('\n');

%% =========================================================================
%  SECTION 14: BATTERY SIZING
%  Tattu 4S 10000mAh 15C LiPo
%    Rated capacity : 148 Wh (14.8V × 10Ah)
%    Usable (80% DoD): 148 × 0.80 = 118.4 Wh
%
%  Battery role: dawn/dusk buffer and peak-demand buffer
%  Solar generation covers daytime consumption with large surplus
%  Battery must cover night standby independently
%
%  Night energy demand = P_standby × 12h = 4 × 12 = 48 Wh
%  Usable battery = 118.4 Wh → covers night with 70.4 Wh margin
% =========================================================================

Bat_rated_Wh  = 148.0;    % [Wh]  Tattu 4S 10000mAh at 14.8V
Bat_usable_Wh = Bat_rated_Wh * DoD_max;  % [Wh]  = 118.4 Wh

% Check night coverage
night_deficit = E_night - Bat_usable_Wh;
if night_deficit > 0
    fprintf('WARNING: Battery insufficient for night standby! Deficit: %.1f Wh\n', night_deficit);
else
    night_margin = Bat_usable_Wh - E_night;
end

fprintf('[14] Battery Analysis\n');
fprintf('    Tattu 4S 10000mAh LiPo\n');
fprintf('    Rated capacity         : %.1f Wh\n', Bat_rated_Wh);
fprintf('    Usable (80%% DoD)       : %.1f Wh\n', Bat_usable_Wh);
fprintf('    Night standby demand   : %.1f Wh\n', E_night);
fprintf('    Night coverage margin  : %.1f Wh  (%.0f%% spare)\n', ...
        Bat_usable_Wh - E_night, (Bat_usable_Wh - E_night)/Bat_usable_Wh*100);
fprintf('\n');

%% =========================================================================
%  SECTION 15: NET ENERGY BALANCE (Net-Zero Verification)
% =========================================================================

E_net = E_solar_total - E_total_consumed;

fprintf('[15] Net Energy Balance — Net-Zero Verification\n');
fprintf('    ─────────────────────────────────────────\n');
fprintf('    Total solar generation  : %8.1f Wh\n', E_solar_total);
fprintf('    Total energy consumed   : %8.1f Wh\n', E_total_consumed);
fprintf('    NET BALANCE             : %8.1f Wh\n', E_net);
if E_net >= 0
    fprintf('    STATUS: NET-POSITIVE ✓  (surplus %.0f Wh = %.0f%% margin)\n', ...
            E_net, E_net/E_total_consumed*100);
else
    fprintf('    STATUS: ENERGY DEFICIT ✗  — increase solar area or reduce consumption\n');
end
fprintf('\n');

%% =========================================================================
%  SECTION 16: RL AGENT — STATE SPACE & ACTION SPACE DEFINITION
%  This section formalises the RL problem for the buoyancy shock control
% =========================================================================

fprintf('[16] RL Agent — Buoyancy Shock Control\n');
fprintf('    State space (observations):\n');
fprintf('      s1: Vertical acceleration   [m/s²]  — IMU (ICM-42688-P on Pixhawk)\n');
fprintf('      s2: Vertical velocity        [m/s]  — integrated from IMU\n');
fprintf('      s3: Altitude error           [m]    — barometer MS5611 vs target\n');
fprintf('      s4: Pitch angle              [deg]  — Pixhawk EKF output\n');
fprintf('      s5: Yaw angle                [deg]  — Pixhawk EKF output\n');
fprintf('      s6: Winch motor current      [A]    — Cytron driver feedback\n');
fprintf('      s7: Winch rope length        [m]    — encoder count integration\n');
fprintf('    \n');
fprintf('    Action space (outputs, continuous):\n');
fprintf('      a1: Vertical motor PWM       [0–1]  — MN3110 downward thrust\n');
fprintf('      a2: Elevator deflection      [deg]  — HS-65HB, range ±30°\n');
fprintf('      a3: Port motor delta         [%%]   — MN3110 port differential\n');
fprintf('      a4: Stbd motor delta         [%%]   — MN3110 stbd differential\n');
fprintf('    \n');
fprintf('    Reward function (per timestep):\n');
fprintf('      r = -|Δalt| × w_alt - |pitch| × w_pitch - |yaw_rate| × w_yaw\n');
fprintf('      w_alt=2.0, w_pitch=0.5, w_yaw=0.3  (altitude primary objective)\n');
fprintf('    \n');
fprintf('    Anticipation logic:\n');
fprintf('      Winch current drops below threshold → RL takes control\n');
fprintf('      (200–400ms before IMU registers shock → proactive response)\n');
fprintf('\n');

%% =========================================================================
%  SECTION 17: SUMMARY TABLE
% =========================================================================

fprintf('=================================================================\n');
fprintf(' FINAL SIZING SUMMARY\n');
fprintf('=================================================================\n');
fprintf('  Hull shape          : Prolate Spheroid, k = %.1f\n', k);
fprintf('  Hull volume         : %.3f m³\n', V_eq);
fprintf('  Hull length         : %.3f m\n', L_eq);
fprintf('  Hull diameter       : %.3f m\n', D_eq);
fprintf('  Envelope area       : %.3f m²\n', S_eq);
fprintf('  Total system mass   : %.3f kg\n', m_total);
fprintf('    Envelope          : %.3f kg\n', m_env_eq);
fprintf('    Hardware (BOM)    : %.3f kg\n', M_hw);
fprintf('    Payload           : %.3f kg\n', m_payload);
fprintf('  Drag at cruise      : %.3f N\n', D_drag);
fprintf('  Cruise power (elec) : %.2f W  (both rear motors)\n', P_cruise_elec);
fprintf('  Avionics power      : %.1f W\n', P_avionics);
fprintf('  Total cruise power  : %.2f W\n', P_mode_cruise);
fprintf('  Solar peak power    : %.1f W\n', eta_solar*eta_MPPT*A_solar*G_peak);
fprintf('  Solar energy/day    : %.1f Wh\n', E_solar_total);
fprintf('  Total consumed/day  : %.1f Wh\n', E_total_consumed);
fprintf('  Net energy balance  : %.1f Wh  (%s)\n', E_net, ...
        conditional_str(E_net >= 0, 'NET POSITIVE ✓', 'DEFICIT ✗'));
fprintf('=================================================================\n\n');

%% =========================================================================
%  SECTION 18: PLOTS
% =========================================================================

figure('Name','SURA Airship — Simulation Results','NumberTitle','off', ...
       'Position',[100 100 1400 900]);

% --- Plot 1: Solar irradiance and power ---
subplot(2,3,1);
yyaxis left
plot(t_vec, G_t, 'Color',[1 0.6 0], 'LineWidth', 2);
ylabel('Irradiance G(t)  [W/m²]');
ylim([0 1100]);
yyaxis right
plot(t_vec, P_solar_t, 'Color',[0.1 0.7 0.1], 'LineWidth', 2);
ylabel('Solar Power P_{solar}  [W]');
xlabel('Time of Day  [h]');
title('Solar Irradiance & Harvested Power');
xlim([0 24]);
xticklabels({'00:00','04:00','08:00','12:00','16:00','20:00','24:00'});
xticks(0:4:24);
grid on; legend('Irradiance','P_{solar}','Location','northwest');

% --- Plot 2: Cumulative energy ---
E_cumulative_solar    = cumtrapz(t_vec, P_solar_t);
% Build consumed power profile
P_consumed_t = zeros(size(t_vec));
for i = 1:length(t_vec)
    ti = t_vec(i);
    if ti >= 6 && ti <= 14    % cruise morning
        P_consumed_t(i) = P_mode_cruise;
    elseif ti > 14 && ti <= 16 % delivery window
        P_consumed_t(i) = P_mode_hover;
    elseif ti > 16 && ti <= 18 % cruise return
        P_consumed_t(i) = P_mode_cruise;
    else
        P_consumed_t(i) = P_mode_night;
    end
end
E_cumulative_consumed = cumtrapz(t_vec, P_consumed_t);

subplot(2,3,2);
plot(t_vec, E_cumulative_solar, 'g-', 'LineWidth', 2); hold on;
plot(t_vec, E_cumulative_consumed, 'r-', 'LineWidth', 2);
fill([t_vec, fliplr(t_vec)], ...
     [E_cumulative_solar, fliplr(E_cumulative_consumed)], ...
     'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
xlabel('Time of Day  [h]');
ylabel('Cumulative Energy  [Wh]');
title('Cumulative Energy Balance');
xlim([0 24]); xticks(0:4:24);
xticklabels({'00:00','04:00','08:00','12:00','16:00','20:00','24:00'});
legend('Solar Generated','Consumed','Surplus','Location','northwest');
grid on;

% --- Plot 3: Power consumption by mode ---
subplot(2,3,3);
modes     = {'Cruise','Hover','Shock','Winch','Night'};
energies  = [E_cruise, E_hover, E_shock, E_winch, E_night];
colors    = [0.2 0.4 0.8; 0.3 0.7 0.3; 0.9 0.3 0.3; 0.8 0.6 0.1; 0.5 0.5 0.5];
b = bar(energies, 'FaceColor', 'flat');
b.CData = colors;
set(gca, 'XTickLabel', modes);
ylabel('Energy  [Wh]');
title('Energy Consumption by Mission Mode');
grid on;
for i = 1:length(energies)
    text(i, energies(i)+2, sprintf('%.1f', energies(i)), ...
         'HorizontalAlignment','center','FontSize',9);
end

% --- Plot 4: Mass budget pie ---
subplot(2,3,4);
mass_cats   = {'Envelope','Propulsion','Control','Computers', ...
               'Navigation','Power Sys','Payload Mech','Structural','Payload'};
mass_vals   = [m_env_eq, m_propulsion, m_control, m_computers, ...
               m_navigation, m_power, m_payload_mech, m_structural, m_payload];
pie_colors  = lines(length(mass_vals));
pie(mass_vals);
legend(mass_cats, 'Location','bestoutside','FontSize',7);
title(sprintf('Mass Budget  (Total: %.3f kg)', m_total));

% --- Plot 5: Buoyancy shock power profile (single event) ---
subplot(2,3,5);
t_event = linspace(0, 60, 1000);   % 60 seconds around event
% RL motor response: pre-arm at t=-5s, ramp up, arrest, ease off
P_vert_profile = zeros(size(t_event));
% Pre-arm window (RL detects winch current drop at t=0, acts at t=0)
% Ramp up over 0.5s, hold 25s, ramp down over 5s
for i = 1:length(t_event)
    tt = t_event(i) - 5;  % shift so shock occurs at t=5s
    if tt < -2
        P_vert_profile(i) = 5;   % pre-arm (spinning at low %)
    elseif tt < 0
        P_vert_profile(i) = 5 + (P_vert_peak-5) * (tt+2)/2;  % ramp up
    elseif tt < 25
        P_vert_profile(i) = P_vert_peak;  % hold
    elseif tt < 35
        P_vert_profile(i) = P_vert_peak * (1-(tt-25)/10);  % ramp down
    else
        P_vert_profile(i) = 0;
    end
end
plot(t_event-5, P_vert_profile, 'b-', 'LineWidth', 2); hold on;
xline(0, 'r--', 'Payload Release', 'LineWidth', 1.5, 'LabelVerticalAlignment','bottom');
xline(-2, 'g--', 'RL Anticipates', 'LineWidth', 1.5, 'LabelVerticalAlignment','bottom');
xlabel('Time relative to payload release  [s]');
ylabel('Vertical Motor Power  [W]');
title('RL Buoyancy Shock Response — Vertical Motor');
ylim([0 55]); grid on;
text(5, 44, sprintf('Peak: %.0fW\nEnergy: %.3f Wh/event', P_vert_peak, P_vert_peak*30/3600), ...
     'FontSize', 9, 'BackgroundColor', 'white');

% --- Plot 6: Hull cross-section (prolate spheroid) ---
subplot(2,3,6);
theta  = linspace(0, 2*pi, 300);
phi    = linspace(-pi/2, pi/2, 300);
x_hull = a_eq * cos(phi);
y_hull = b_eq * sin(phi);
plot(x_hull, y_hull, 'b-', 'LineWidth', 2.5); hold on;
plot(x_hull, -y_hull, 'b-', 'LineWidth', 2.5);
fill([x_hull, fliplr(x_hull)], [y_hull, fliplr(-y_hull)], ...
     [0.7 0.85 1.0], 'FaceAlpha', 0.4, 'EdgeColor','none');
% Gondola
rectangle('Position', [-0.3, -(b_eq+0.25), 0.6, 0.2], ...
          'Curvature',[0.3 0.3], 'FaceColor',[0.6 0.4 0.2], 'EdgeColor','k');
% Solar panels on top
x_panel = linspace(-0.7, 0.7, 10);
y_panel = arrayfun(@(x) b_eq*sqrt(1-(x/a_eq)^2), x_panel);
scatter(x_panel, y_panel, 40, 'g', 'filled');
% Motor indicators
scatter([a_eq*0.85, a_eq*0.85], [b_eq*0.3, -b_eq*0.3], 60, 'r', 'filled');
scatter([0], [-(b_eq+0.05)], 60, [0.5 0 0.5], 'filled');
text(a_eq*0.87, b_eq*0.3+0.1, 'Port Motor', 'FontSize',8);
text(a_eq*0.87, -b_eq*0.3-0.15, 'Stbd Motor', 'FontSize',8);
text(0.05, -(b_eq+0.05), 'Vert Motor', 'FontSize',8, 'Color',[0.5 0 0.5]);
axis equal; grid on;
xlabel('Longitudinal axis  [m]');
ylabel('Lateral axis  [m]');
title(sprintf('Hull Profile  (L=%.2fm, D=%.2fm)', L_eq, D_eq));
legend({'Upper hull','Lower hull','Helium envelope','Solar panels', ...
        'Cruise motors','Vertical motor'}, 'Location','southwest','FontSize',7);

sgtitle('SURA Airship 2025 — Complete Sizing & Simulation Results', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% =========================================================================
%  SECTION 19: PARAMETER SENSITIVITY ANALYSIS
%  Sweep fineness ratio k and solar efficiency to check robustness
% =========================================================================

k_range      = 3.0:0.1:5.0;
eta_range    = 0.08:0.01:0.14;
V_k          = zeros(size(k_range));
E_net_eta    = zeros(size(eta_range));

for i = 1:length(k_range)
    ki = k_range(i);
    CDv_i = 0.026 + 0.004*(ki-3.5)^2 + 0.001*(ki-5);  % rough CDv(k) curve
    f_k = @(V) mass_balance_fn(V, rho_air, rho_He, sigma_env, ki, M_hw, m_payload);
    try
        V_k(i) = fzero(f_k, V_guess, options);
    catch
        V_k(i) = NaN;
    end
end

for i = 1:length(eta_range)
    E_solar_i  = eta_range(i) * eta_MPPT * A_solar * G_peak * t_day * 2/pi;
    E_net_eta(i) = E_solar_i - E_total_consumed;
end

figure('Name','Sensitivity Analysis','NumberTitle','off','Position',[150 150 900 400]);

subplot(1,2,1);
plot(k_range, V_k, 'b-o', 'LineWidth',2,'MarkerSize',5);
xline(3.5, 'r--', 'k=3.5 (design)', 'LineWidth',1.5);
xlabel('Fineness Ratio k'); ylabel('Required Volume  [m³]');
title('Sensitivity: Fineness Ratio vs Volume');
grid on;

subplot(1,2,2);
plot(eta_range*100, E_net_eta, 'g-o', 'LineWidth',2,'MarkerSize',5);
xline(eta_solar*100, 'r--', sprintf('η=%.0f%% (design)',eta_solar*100), 'LineWidth',1.5);
yline(0, 'k-', 'Net zero', 'LineWidth',1);
xlabel('Solar Efficiency  [%]'); ylabel('Net Energy Balance  [Wh]');
title('Sensitivity: Solar Efficiency vs Net Energy');
grid on;

sgtitle('SURA Airship 2025 — Parameter Sensitivity Analysis', ...
        'FontSize',13,'FontWeight','bold');

fprintf('[18-19] Plots generated.\n');
fprintf('        Figure 1: Sizing & Simulation Results (6 subplots)\n');
fprintf('        Figure 2: Sensitivity Analysis (k and eta_solar sweeps)\n\n');
fprintf('=================================================================\n');
fprintf(' SIMULATION COMPLETE\n');
fprintf('=================================================================\n');

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function f = mass_balance_fn(V, rho_air, rho_He, sigma_env, k, M_hw, m_payload)
    % Prolate spheroid geometry from volume V and fineness ratio k
    % V = (4/3)*pi*k*b^3  →  b = (3V/(4*pi*k))^(1/3)
    b = (3*V / (4*pi*k))^(1/3);
    a = k * b;
    % Eccentricity
    e = sqrt(1 - (b/a)^2);
    % Surface area (exact prolate spheroid formula)
    S = 2*pi*b^2 * (1 + (a/b) * asin(e)/e);
    % Envelope mass
    m_env = sigma_env * S;
    % Mass balance: net lift - total weight = 0
    f = (rho_air - rho_He)*V - m_env - M_hw - m_payload;
end

function s = conditional_str(condition, s_true, s_false)
    if condition
        s = s_true;
    else
        s = s_false;
    end
end