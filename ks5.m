%% =========================================================================
%  SURA AIRSHIP — COMPLETE SIZING & ENERGY SIMULATION SCRIPT  v3.0
%  Agya Sanghi & Kartik Aggarwal | IIT Delhi | SURA 2025
%
%  Airship Type   : Autonomous Solar-Assisted LTA Delivery Airship
%  Hull Shape     : Prolate Spheroid (fineness ratio k = 4.0)
%  Payload        : 2 kg, winch-deployed (Yo-Yo manoeuvre)
%  Mission        : 12-hour net-zero diurnal cycle
%
%  ACTUATOR CONFIGURATION (8 total — rationalised from v2):
%    Motor 1 : T-Motor MN3110 KV470       — Port rear cruise
%    Motor 2 : T-Motor MN3110 KV470       — Starboard rear cruise
%    Motor 3 : T-Motor MN3110 KV470       — Vertical (RL buoyancy shock)
%    Motor 4 : Pololu 37D 19:1 Gear Motor — Winch (payload deploy/retrieve)
%    Servo 1 : Hitec HS-65HB              — Elevator (pitch control)
%    Servo 2 : Hitec HS-65HB              — Rudder (yaw control)
%    Servo 3 : Hitec HS-65HB              — Ballonet INTAKE valve
%    Servo 4 : Hitec HS-65HB              — Ballonet EXHAUST valve
%
%  CHANGES FROM v2 (with full reasoning):
%  ─────────────────────────────────────────────────────────────────────
%  [C1] k = 3.5 → k = 4.0
%       Reason: Khoury & Gillett "Airship Technology" Ch.2 and AIAA-2002-3932
%       (Lutz & Munson) both show CDv is minimised at k ≈ 4.0 for prolate
%       spheroids at Re > 2×10^5. k=3.5 is 6-8% sub-optimal for drag.
%       CDv updated to 0.025 (from 0.030) using Khoury Table 2.1.
%
%  [C2] CDv = 0.030 → CDv = 0.025
%       Reason: At k=4.0, Khoury & Gillett Table 2.1 gives CDv = 0.024–0.026.
%       The value 0.030 is correct at k=3.5 but not at k=4.0.
%       Using 0.025 (midpoint, conservative). Not Hoerner — Hoerner's CDv
%       formula is for skin friction only (flat plate analogy) and gives ~0.18,
%       which is WRONG for LTA use where CDv is referenced to V^(2/3) and
%       includes full pressure drag + base drag for a streamlined body.
%
%  [C3] Solar irradiance: sin(π(t-6)/12) → astronomical cosine model
%       Reason: The simple sine model has two errors:
%         (a) It assumes sunrise at exactly 06:00 and sunset at 18:00.
%             In Delhi (28.6°N) at summer solstice, sunrise is ~05:06 and
%             sunset is ~19:12 — a 13.2-hour solar window, not 12 hours.
%         (b) The sine shape is mathematically convenient but physically
%             incorrect. The correct irradiance on a horizontal surface is
%             G(t) = G_peak × cos(θz(t)) where θz is the solar zenith angle.
%             This uses the full astronomical formula (hour angle, latitude,
%             declination — Spencer 1971, Meeus "Astronomical Algorithms").
%         (c) The sine model underestimates morning/evening irradiance
%             (e.g. at 07:00, sine gives 259 W/m² vs cosine gives 359 W/m²)
%             and overestimates peak by forcing G_peak=1000 regardless of
%             solar elevation angle.
%       The corrected model uses G(t) = G_clearsky × max(0, cos(θz(t)))
%       where cos(θz) = sin(φ)sin(δ) + cos(φ)cos(δ)cos(h).
%       G_clearsky = 900 W/m² (IMD Delhi clear-sky summer measured peak GHI).
%       Net effect: slightly more total daily energy (+1.5%) but more
%       physically accurate slope — important for SoC simulation.
%
%  [C4] Ballonet volume: CRITICAL ERROR FIXED
%       v2 computed V_ballonet_max = 8.38 m³ for 4 × 2kg drops.
%       This exceeds the hull volume (~6 m³) — physically impossible.
%       ROOT CAUSE: to compensate for 2 kg of mass loss you need
%       2/1.14 = 1.75 m³ of air per drop. Four drops = 7.0 m³.
%       A 6 m³ hull cannot contain 7 m³ of air inside it.
%       CORRECT DESIGN:
%         The ballonet is sized at 25% of hull volume (industry standard,
%         Khoury & Gillett Ch.5: "ballonets typically 20-30% of hull volume").
%         25% of ~6 m³ = 1.5 m³ → can add 1.5 × 1.14 = 1.71 kg of air.
%         This compensates for approximately ONE 2 kg drop per fill cycle.
%         Between deliveries, the PID altitude hold vents the ballonet,
%         then refills before the next drop. This is the operationally
%         correct mode — not filling cumulatively across all drops.
%       The ballonet exists primarily for PRESSURE MANAGEMENT (envelope
%       pressure relative to ambient as altitude changes) and TRIM.
%       It is NOT the primary post-drop buoyancy corrector — the RL motor
%       handles the transient (30s), then altitude hold + ballonet handle
%       the steady-state trim between deliveries.
%
%  [C5] Ballonet fan: 30mm micro blower → Micronel U50L 50mm blower
%       Reason: 30mm fan flow = 3 L/min = 0.003 m³/min.
%       To fill 1.5 m³ ballonet: time = 1.5/0.003 = 500 minutes. Unusable.
%       Micronel U50L: 400 L/min = 0.4 m³/min → 1.5/0.4 = 3.75 min. Acceptable.
%       Mass: 120g, Power: 15W at 24V. Source: micronel.com.
%
%  [C6] Solar panel area: 2.0 m² → 1.82 m² (physical area, 8 × 25W HL-25 modules)
%       Reason: Each Ascent HL-25 module: 25W / (1000 × 0.11) = 0.2273 m².
%       8 modules = 1.818 m². The v1 value of 2.0 m² had no physical basis.
%
%  [C7] All intermediate loopholes from v2 (L1–L10) retained and documented.
%
%  REFERENCES:
%    [R1] Khoury & Gillett, "Airship Technology", Cambridge Univ Press, 2004
%    [R2] Lutz & Munson, AIAA-2002-3932 "Numerical investigations of the flow
%         around the LOTTE airship" — CDv vs k table
%    [R3] Spencer, J.W. (1971), "Fourier series representation of the position
%         of the sun", Search 2(5) — solar position formula
%    [R4] Kasten & Young (1989), "Revised optical air mass tables" — AM formula
%    [R5] IMD Pune, "Solar radiation atlas of India" — G_clearsky = 900 W/m²
%    [R6] Duffie & Beckman, "Solar Engineering of Thermal Processes", 4th ed.
%    [R7] Battery University BU-409 — LiPo CC/CV efficiency
%    [R8] NIST Webbook (webbook.nist.gov) — He density at STP
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf(' SURA AIRSHIP v3.0 — PHYSICS-BASED SIZING & ENERGY SIMULATION\n');
fprintf(' Agya Sanghi & Kartik Aggarwal | IIT Delhi | SURA 2025\n');
fprintf('=================================================================\n\n');

%% =========================================================================
%  SECTION 1: PHYSICAL CONSTANTS
%  ─ All values from NIST webbook or ISA standard
%  ─ Operating conditions set to Indian summer (35°C) — NOT ISA 15°C
%    because lift is lower in hot air, this is the CONSERVATIVE approach
% =========================================================================

g        = 9.81;          % [m/s²]   gravitational acceleration
R_He     = 2077.0;        % [J/kg·K] specific gas constant, helium (NIST [R8])
R_air    = 287.053;       % [J/kg·K] specific gas constant, dry air (ISA)
T_amb    = 308.15;        % [K]      35°C — Indian summer operating temperature
P_atm    = 101325;        % [Pa]     sea-level ISA pressure

% Air and helium density at operating temperature (35°C)
% Using ideal gas law: ρ = P/(R·T) — valid for both at these conditions
rho_air_op = P_atm / (R_air * T_amb);   % 1.1455 kg/m³ at 35°C
rho_He_op  = P_atm / (R_He * T_amb);   % 0.1576 kg/m³ at 35°C

% Helium superheat correction (Khoury & Gillett Ch.1 [R1]):
%  - Solar radiation heats the gas inside the envelope
%  - Conservative estimate: +10°C above ambient (not +15° which gives too much lift)
%  - Superheat REDUCES rho_He → REDUCES density difference → REDUCES lift margin
%  - This is conservative because it makes sizing harder, not easier
T_He = T_amb + 10.0;                   % [K]  319.15 K during daytime
rho_He_hot = P_atm / (R_He * T_He);   % 0.1526 kg/m³ (slightly lower than rho_He_op)

% Net buoyancy density = what each m³ of He-filled volume weighs in air minus gas
delta_rho = rho_air_op - rho_He_hot;  % 0.9929 kg/m³  ← used in ALL mass balance

fprintf('[1] Physical Constants\n');
fprintf('    Ambient temperature  : %.1f°C  (%.2f K)\n', T_amb-273.15, T_amb);
fprintf('    rho_air (35°C)       : %.4f kg/m³\n', rho_air_op);
fprintf('    rho_He  (35°C+10K)   : %.4f kg/m³ (with solar superheat)\n', rho_He_hot);
fprintf('    delta_rho (net lift) : %.4f kg/m³  (per m³ of helium)\n\n', delta_rho);

%% =========================================================================
%  SECTION 2: HULL GEOMETRY — PROLATE SPHEROID
%
%  Fineness ratio k = a/b (length/diameter ratio)
%    k = 4.0 chosen because:
%    (a) CDv is minimised near k=4 (Khoury Table 2.1, Lutz & Munson AIAA [R2])
%    (b) k < 3 → fat body, very high CDv, cannot cruise efficiently
%    (c) k > 5 → longer body, higher wetted area, structural challenges
%    (d) k=4 is the sweet spot universally used in modern LTA design
%
%  CDv = 0.025 for k = 4.0:
%    - Source: Khoury & Gillett Table 2.1 gives 0.024–0.026 at k=4
%    - AIAA-2002-3932 (Lutz & Munson) confirms 0.022–0.026 numerically
%    - Using 0.025 (upper half of range = conservative)
%    - CDv is referenced to V^(2/3) — volumetric form of drag coefficient
%      (distinct from Cd which is referenced to frontal area)
%    - IMPORTANT: Do NOT use Hoerner's flat-plate formula for CDv —
%      it yields ~0.18, which is valid for skin friction only and is
%      referenced differently. The LTA volumetric CDv is an experimentally
%      fitted result for shaped bodies at high Reynolds numbers.
% =========================================================================

k   = 4.0;    % fineness ratio, a/b
CDv = 0.025;  % volumetric drag coefficient (Khoury Table 2.1, k=4) [R1][R2]

fprintf('[2] Hull Geometry\n');
fprintf('    Shape            : Prolate Spheroid (optimum k for LTA cruise)\n');
fprintf('    Fineness ratio k : %.1f  (Khoury & Gillett — CDv minimum at k≈4)\n', k);
fprintf('    CDv              : %.3f  (Khoury Table 2.1: range 0.024–0.026 at k=4)\n\n', CDv);

%% =========================================================================
%  SECTION 3: ENVELOPE MATERIAL
%
%  PET/Mylar laminate, 0.070 kg/m²
%    - Lindstrand / Aerostar commercial blimp skin datasheet
%    - Heavy-duty, UV-resistant, outdoor use grade
%    - He permeability: ~1 L/m²/day (industry measured for PET laminates)
%      → ~5–7 m²hull × 1 L/m²/day = 5–7 litres/day loss
%      → Mandatory weekly helium top-up for sustained operation
%    - Tensile strength: adequate for this volume (P_internal ~50–100 Pa gauge)
% =========================================================================

sigma_env = 0.070;   % [kg/m²] PET/Mylar laminate (Lindstrand/Aerostar spec)

fprintf('[3] Envelope Material: PET/Mylar laminate, 0.070 kg/m²\n');
fprintf('    He permeability: ~1 L/m²/day → top-up required weekly.\n\n');

%% =========================================================================
%  SECTION 4: HARDWARE MASS BUDGET
%  All individual masses from manufacturer datasheets (see urls in comments)
%  Summed bottom-up — NO arbitrary placeholders
% =========================================================================

% --- PROPULSION ---
% T-Motor MN3110 KV470 (store.tmotor.com):
%   - 80g body + ~19g cables/connector = 99g listed mass
%   - 3 units: port cruise, starboard cruise, vertical (RL shock)
m_MN3110         = 0.099;              % [kg] per motor
m_motors_flight  = m_MN3110 * 3;      % 3x: 0.297 kg

% Pololu 37D 19:1 Gear Motor (pololu.com):
%   - 12V, 300 RPM no-load, stall torque 7.5 kg·cm, 160g
%   - Rope load: m_payload × g = 2×9.81 = 19.62 N → torque at winch = 19.62×0.015m = 0.29 N·m
%   - 19:1 gear: motor torque needed = 0.29/19 = 0.015 N·m → ~35% rated → comfortable
m_motor_winch    = 0.160;             % [kg]

% T-Motor AM 30A BLHeli_32 ESC (store.tmotor.com):
%   - 28g each, 2–6S, supports telemetry — one per MN3110
m_ESC_30A        = 0.028;
m_ESCs_flight    = m_ESC_30A * 3;    % 0.084 kg

% Cytron MD13S 13A DC Motor Driver (cytron.io):
%   - Bidirectional, 6–30V, 30g, supports PWM + DIR from Pixhawk AUX
m_ESC_winch      = 0.030;            % [kg]

% T-Motor 15×5 CF prop pair (store.tmotor.com):
%   - 32g per pair. One pair per MN3110.
m_prop_pair      = 0.032;
m_props          = m_prop_pair * 3;  % 0.096 kg

m_propulsion = m_motors_flight + m_motor_winch + m_ESCs_flight + m_ESC_winch + m_props;

% --- CONTROL SURFACES ---
% Hitec HS-65HB (hitecrcd.com): 9.1g, 4.8V, 1.5 kg·cm torque
% 4 servos: elevator, rudder, ballonet intake valve, ballonet exhaust valve
m_HS65HB     = 0.009;               % [kg]
n_servos     = 4;
m_servos     = m_HS65HB * n_servos; % 0.036 kg

% 4x tail fins: 3mm CF rod + EPP foam sheets, ~10g each
% Aerodynamic sizing: fin area ~ 5% of hull planform area (LTA rule of thumb)
m_tail_fins  = 0.040;               % [kg]

m_control    = m_servos + m_tail_fins;  % 0.076 kg

% --- FLIGHT COMPUTERS ---
% Holybro Pixhawk 6C Mini (holybro.com): STM32H743VIH6, ArduBlimp-supported
m_pixhawk    = 0.032;   % [kg]
% Raspberry Pi 4B 4GB (raspberrypi.com): 46g, runs YOLOv8 + ROS 2 Nav2
m_rpi4       = 0.046;   % [kg]
% RPi AI Camera (raspberrypi.com): IMX500 on-sensor inference, 14g
m_ai_cam     = 0.014;   % [kg]

m_computers  = m_pixhawk + m_rpi4 + m_ai_cam;  % 0.092 kg

% --- NAVIGATION & SENSING ---
% Holybro M10 GPS + IST8310 magnetometer (holybro.com): uBlox M10, 36g with mast
m_GPS        = 0.036;   % [kg]
% Holybro SiK Telemetry v3 433MHz (holybro.com): 36g air module, 1W TX, 300m+ range
m_telemetry  = 0.036;   % [kg]
% MS5611 barometric altimeter: integrated inside Pixhawk — zero extra mass

m_navigation = m_GPS + m_telemetry;  % 0.072 kg

% --- POWER SYSTEM ---
% Tattu 4S 10000mAh 15C LiPo (genstattu.com): 620g, 14.8V, 148 Wh
m_battery    = 0.620;   % [kg]
% Ascent Solar HL-25 HyperLight CIGS 25W × 8 (pv-magazine.com): 82g each
% 8 modules chosen to fill the hull top: 8 × 0.2273 m² = 1.818 m² total area
m_solar_mod  = 0.082;
n_solar      = 8;
m_solar      = m_solar_mod * n_solar;  % 0.656 kg
% Genasun GVB-8 LiPo MPPT (genasun.com): 57g, 5–35V input, 8A
m_MPPT       = 0.057;   % [kg]
% Matek HUBOSD8 PDB (mateksys.com): 14g, 5V/12V BECs
m_PDB        = 0.014;   % [kg]

m_power      = m_battery + m_solar + m_MPPT + m_PDB;  % 1.347 kg

% --- PAYLOAD MECHANISM ---
% 1mm Dyneema SK75 cord: 1.6 g/m × 20m = 32g + spool ~3g = 35g
m_rope       = 0.035;   % [kg]
% 30mm Al auto-locking carabiner: 14g (standard climbing hardware)
m_carabiner  = 0.014;   % [kg]
% Spring-loaded mechanical latch: 15g, no servo required
% Gravity drop: gondola at bottom → payload weight + winch reversal trips latch
m_latch      = 0.015;   % [kg]

m_payload_mech = m_rope + m_carabiner + m_latch;  % 0.064 kg

% --- BALLONET SYSTEM ---
% DESIGN RATIONALE (see [C4] in header for full explanation):
%
%  A ballonet is an air-filled bladder inside the helium envelope.
%  REAL PURPOSE in this airship:
%  (1) Envelope pressure management: as altitude increases, ambient
%      pressure drops → helium would expand and over-pressurize envelope.
%      Venting ballonet air prevents this. CRITICAL for envelope integrity.
%  (2) Post-drop trim: after releasing 2kg payload, pump air in to add
%      ~2kg mass back, restoring neutral buoyancy at cruise altitude.
%      One fill cycle = one delivery = one drop + refill before next drop.
%
%  SIZING RULE:
%  Industry standard: ballonet = 20–30% of hull volume [R1] Ch.5.
%  We use 25%. Hull volume is ~6 m³ (solved in Section 11).
%  V_ballonet = 0.25 × V_hull (set iteratively after Section 11).
%  For initial hardware mass estimate: use V_ballonet_prelim = 0.25 × 6 = 1.5 m³.
%
%  Air mass in full ballonet: 1.5 × 1.14 = 1.71 kg ≈ payload mass (2 kg).
%  This is intentional: one full-fill compensates one 2 kg drop.
%  Between deliveries: PID vents ballonet, refills before next drop.

V_ballonet_frac = 0.25;   % ballonet as fraction of hull volume (industry standard)
V_hull_prelim   = 6.0;    % [m³] preliminary estimate (refined after fzero)
V_ballonet_prelim = V_ballonet_frac * V_hull_prelim;  % 1.5 m³

% Bladder (TPU-coated nylon, 0.035 kg/m²):
%   Bladder modelled as sphere for surface area estimate
r_bladder_prelim = (3*V_ballonet_prelim / (4*pi))^(1/3);  % 0.709 m
S_bladder_prelim = 4*pi*r_bladder_prelim^2;               % 6.32 m²
m_bladder        = 0.035 * S_bladder_prelim;              % 0.221 kg

% Blower fan (Micronel U50L-024KX-4 or equivalent 50mm centrifugal blower):
%   - Flow: 400 L/min = 0.4 m³/min
%   - Fill time: 1.5 m³ / 0.4 = 3.75 min per cycle ← acceptable
%   - 30mm fan was WRONG: 3 L/min → 500 min per cycle → unusable
%   - Power: 15W at 24V (fan runs during inflation only, not continuously)
%   - Mass: 120g. Source: micronel.com
m_ballonet_fan   = 0.120;  % [kg]
P_ballonet_fan   = 15.0;   % [W]
V_fan_flow       = 0.400;  % [m³/min] rated flow

% Silicone duct + intake filter (connects fan to bladder through hull fitting)
m_ballonet_duct  = 0.010;  % [kg]
% Hull penetration fittings × 2 (aluminium barbed push-fit, 4g each)
m_ballonet_fit   = 0.008;  % [kg]
% Valve gate mechanisms (3D-printed PLA, ~3g each × 2): included in gondola mass

m_ballonet = m_bladder + m_ballonet_fan + m_ballonet_duct + m_ballonet_fit;  % 0.359 kg

% --- STRUCTURAL & MISC ---
% Wiring: XT60 plugs, 16AWG silicone wire, JST-XH connectors
m_wiring   = 0.080;  % [kg]
% Gondola: 3D-printed PLA + CF tube frame (houses all electronics)
m_gondola  = 0.120;  % [kg]
% Vibration standoffs: 8× silicone mounts for Pixhawk isolation
m_dampers  = 0.010;  % [kg]

m_structural = m_wiring + m_gondola + m_dampers;  % 0.210 kg

% --- TOTAL HARDWARE MASS ---
M_hw = m_propulsion + m_control + m_computers + m_navigation + ...
       m_power + m_payload_mech + m_ballonet + m_structural;

fprintf('[4] Hardware Mass Budget (component-level, datasheet-verified)\n');
fprintf('    %-35s : %.3f kg\n', 'Propulsion (3×motor+ESC+prop, winch)', m_propulsion);
fprintf('      %-33s : %.3f kg\n', '3× T-Motor MN3110 KV470', m_motors_flight);
fprintf('      %-33s : %.3f kg\n', 'Pololu 37D 19:1 winch motor', m_motor_winch);
fprintf('      %-33s : %.3f kg\n', '3× T-Motor AM 30A ESC', m_ESCs_flight);
fprintf('      %-33s : %.3f kg\n', 'Cytron MD13S winch driver', m_ESC_winch);
fprintf('      %-33s : %.3f kg\n', '3× T-Motor 15×5 CF props', m_props);
fprintf('    %-35s : %.3f kg\n', 'Control (4× HS-65HB + tail fins)', m_control);
fprintf('      %-33s : %.3f kg\n', '4× Hitec HS-65HB [elev/rud/2×valve]', m_servos);
fprintf('      %-33s : %.3f kg\n', '4× tail fins (CF+EPP)', m_tail_fins);
fprintf('    %-35s : %.3f kg\n', 'Flight Computers (FC+RPi4+AI cam)', m_computers);
fprintf('    %-35s : %.3f kg\n', 'Navigation (M10 GPS + SiK radio)', m_navigation);
fprintf('    %-35s : %.3f kg\n', 'Power (LiPo+solar+MPPT+PDB)', m_power);
fprintf('    %-35s : %.3f kg\n', 'Payload Mech (rope+carabiner+latch)', m_payload_mech);
fprintf('    %-35s : %.3f kg\n', 'Ballonet System (CORRECTED v3)', m_ballonet);
fprintf('      %-33s : %.3f kg\n', 'TPU bladder (25%% hull vol)', m_bladder);
fprintf('      %-33s : %.3f kg\n', 'Micronel 50mm blower fan', m_ballonet_fan);
fprintf('      %-33s : %.3f kg\n', 'Silicone duct + filter', m_ballonet_duct);
fprintf('      %-33s : %.3f kg\n', 'Hull fittings ×2', m_ballonet_fit);
fprintf('    %-35s : %.3f kg\n', 'Structural/Misc (wiring+gondola)', m_structural);
fprintf('    %-35s : %.3f kg\n', '──────────────────────────────', 0.0);
fprintf('    %-35s : %.3f kg\n\n', 'TOTAL M_hw', M_hw);

%% =========================================================================
%  SECTION 5: PAYLOAD
% =========================================================================

m_payload = 2.000;   % [kg] design payload — 2 kg parcel, winch-deployed
n_deliver = 4;       % number of deliveries per mission

fprintf('[5] Payload: %.3f kg (×%d deliveries per mission)\n\n', m_payload, n_deliver);

%% =========================================================================
%  SECTION 6: COMPONENT EFFICIENCIES
%  Every value cited to manufacturer data or published literature.
% =========================================================================

% T-Motor MN3110 KV470 — brushless outrunner efficiency
%   Performance data (store.tmotor.com, MN3110 thrust tables):
%   - Peak efficiency: ~85% at 60% throttle (rated point)
%   - At LTA cruise (~20% throttle): motor eta drops to 78–82%
%   - Use 0.80 (mid-range of cruise-load band, conservative)
eta_motor = 0.80;

% T-Motor 15×5 CF Propeller — propulsive efficiency
%   UIUC Propeller Database (aerospace.illinois.edu):
%   - 15×5 large-diameter slow props: peak eta_prop = 0.80 at optimal advance ratio
%   - At LTA cruise (low airspeed, partial throttle): eta_prop ≈ 0.75
%   - Large diameter is essential for LTA — high mass flow rate, low disc loading
%   - Small high-KV multirotor props would give eta_prop < 0.50 at low speed
eta_prop = 0.75;

% Combined drivetrain (electrical → shaft → thrust)
eta_drive = eta_motor * eta_prop;  % 0.60 = 60%

% Pololu 37D Gear Motor — winch efficiency
%   - DC brush motor at rated load: 65% (Pololu 37D datasheet, pololu.com)
%   - 19:1 metal spur gearbox: 82% (industry standard for metal spur at this ratio)
eta_winch = 0.65 * 0.82;  % 0.533

% Genasun GVB-8 MPPT Solar Charge Controller
%   - Maximum power tracking efficiency: 99.2% (Genasun datasheet, genasun.com)
%   - Buck converter efficiency at rated load: 96.0%
%   - Combined: 0.992 × 0.960 = 0.9523
eta_MPPT = 0.952;

% LiPo Charging Efficiency (CC/CV charging)
%   - Standard LiPo CC/CV charge efficiency: 95% [R7]
%   - Accounts for heat dissipation during constant-current phase
%   - This means: 100 Wh into battery requires 105.3 Wh from solar
eta_charge = 0.950;

% LiPo Discharge Efficiency
%   - LiPo discharge at ~1C rate: 98% (Tattu datasheet, genstattu.com)
eta_discharge = 0.980;

% LiPo Depth of Discharge Limit
%   - Hard limit: never below 20% SoC (cell chemistry protection)
%   - Usable: 80% of rated capacity
DoD_limit = 0.80;

% Solar panel: Ascent HL-25 HyperLight CIGS
%   - Manufacturer rated efficiency: 11.0% (pv-magazine.com, 2021 datasheet)
%   - Thermal derating (India summer, 35°C cell, ~0.4%/°C CIGS TC):
%     CIGS TC is about –0.32%/°C (better than Si). At 35°C above STC (25°C):
%     derating = 1 – 0.0032 × 10 = 0.968 → ~3.2% reduction
%   - Cloud/dust derating for Indian summer: ~8–10% average (IMD data)
%   - Combined derating factor: 0.968 × 0.91 = 0.881 → ~12% total loss
eta_solar_stc  = 0.110;   % 11.0% at STC (rated)
f_thermal      = 0.968;   % CIGS temperature coefficient at 35°C (10K above STC)
f_cloud_dust   = 0.910;   % cloud/dust factor for Delhi summer
eta_solar_eff  = eta_solar_stc * f_thermal * f_cloud_dust;  % 0.0969 ≈ 9.7% effective

% Hull tilt/cosine factor:
%   - Panels bonded conformally to curved hull top
%   - Panels near nose and tail face at angles up to 30° from horizontal
%   - Average cosine loss over curved surface: ~15%
%   - Panels centred on hull top see near-horizontal mounting → less loss
%   - Conservative combined factor: 0.85 [R6]
tilt_factor = 0.85;

% End-to-end solar → battery efficiency chain:
%   eta_solar_eff × tilt_factor × eta_MPPT × eta_charge
eta_solar_system = eta_solar_eff * tilt_factor * eta_MPPT * eta_charge;

fprintf('[6] Component Efficiencies\n');
fprintf('    Motor MN3110 (cruise, 20%% throttle) : %.2f (%.0f%%)\n', eta_motor, eta_motor*100);
fprintf('    Prop 15×5 CF (LTA cruise)           : %.2f (%.0f%%)\n', eta_prop, eta_prop*100);
fprintf('    Combined drivetrain (elec→thrust)   : %.3f (%.1f%%)\n', eta_drive, eta_drive*100);
fprintf('    Winch motor×gear (Pololu 37D)        : %.3f (%.1f%%)\n', eta_winch, eta_winch*100);
fprintf('    MPPT Genasun GVB-8                  : %.3f (%.1f%%)\n', eta_MPPT, eta_MPPT*100);
fprintf('    LiPo CC/CV charge                   : %.3f (%.1f%%)\n', eta_charge, eta_charge*100);
fprintf('    LiPo discharge (~1C)                : %.3f (%.1f%%)\n', eta_discharge, eta_discharge*100);
fprintf('    LiPo max DoD                        : %.0f%%\n', DoD_limit*100);
fprintf('    Solar CIGS rated (STC)              : %.3f (%.1f%%)\n', eta_solar_stc, eta_solar_stc*100);
fprintf('    Thermal derating (35°C, CIGS TC)    : %.3f (%.1f%% loss)\n', f_thermal, (1-f_thermal)*100);
fprintf('    Cloud/dust derating (Delhi summer)  : %.3f (%.1f%% loss)\n', f_cloud_dust, (1-f_cloud_dust)*100);
fprintf('    Effective solar efficiency           : %.4f (%.2f%%)\n', eta_solar_eff, eta_solar_eff*100);
fprintf('    Hull tilt/cosine factor              : %.2f (%.0f%% loss)\n', tilt_factor, (1-tilt_factor)*100);
fprintf('    End-to-end solar→battery            : %.4f (%.2f%%)\n\n', eta_solar_system, eta_solar_system*100);

%% =========================================================================
%  SECTION 7: CRUISE PERFORMANCE
% =========================================================================

v_cruise  = 5.0;   % [m/s]  design cruise airspeed (calm air, energy budget)
v_wind    = 5.0;   % [m/s]  max headwind = Beaufort 3 / gentle breeze boundary
v_eff     = v_cruise + v_wind;  % [m/s] = 10 m/s — worst-case for motor sizing
% Energy budget uses calm-air drag (v_cruise).
% Motor sizing uses headwind drag (v_eff): motor must overcome this or fail.

fprintf('[7] Cruise Performance\n');
fprintf('    Design cruise airspeed      : %.1f m/s\n', v_cruise);
fprintf('    Max headwind (Beaufort 3)   : %.1f m/s\n', v_wind);
fprintf('    Worst-case effective speed  : %.1f m/s (for motor sizing)\n', v_eff);
fprintf('    Energy budget uses          : %.1f m/s (calm air, typical)\n\n', v_cruise);

%% =========================================================================
%  SECTION 8: SOLAR IRRADIANCE MODEL — ASTRONOMICAL COSINE MODEL
%
%  This replaces the simple sin(π(t-6)/12) used in v1/v2.
%  Reasons for the change (see [C3] in header):
%  ─ Sine assumes 12h window (6am–6pm). At Delhi summer, window is 13.8h.
%  ─ Sine assumes symmetric rise/set. Astronomical model includes latitude
%    and declination effects correctly.
%  ─ The slope of the irradiance curve directly affects SoC simulation.
%    Sine has zero irradiance at t=6, but astronomical model gives ~170 W/m²
%    at 6am because the sun is already above the horizon by then.
%
%  FORMULA (Spencer 1971 [R3], Meeus "Astronomical Algorithms"):
%    cos(θz) = sin(φ)·sin(δ) + cos(φ)·cos(δ)·cos(h)
%    where:
%      φ = observer latitude   = 28.64° (IIT Delhi)
%      δ = solar declination   = 23.45° (summer solstice, worst-case for max solar)
%      h = hour angle          = 15°·(t - 12)  [h negative in morning]
%
%  G(t) = G_clearsky × max(0, cos(θz(t)))
%    G_clearsky = 900 W/m²  — IMD measured peak GHI for Delhi summer [R5]
%    (AM1.5 reference is 1000 W/m² but this is for tilted panel facing sun.
%     Horizontal surface peak in Delhi = ~900 W/m². Using this is correct
%     since panels mount approximately horizontal on hull top.)
%
%  G_clearsky = 900 W/m² replaces G_peak = 1000 W/m² from v1/v2.
%  The total daily energy is slightly HIGHER with the new model (+1.5%)
%  because the longer solar window (13.8h vs 12h) more than compensates
%  for the lower peak irradiance. Both models give a similar energy total,
%  validating the approach, but the new model is physically correct.
% =========================================================================

phi_lat   = 28.64 * pi/180;   % [rad] IIT Delhi latitude
delta_sun = 23.45 * pi/180;   % [rad] solar declination at summer solstice
G_clearsky = 900.0;            % [W/m²] peak GHI, Delhi summer (IMD data [R5])

% Time vector: 24h at 1-minute resolution
dt    = 1/60;                  % [h]  time step
t_vec = (0:dt:24);             % [h]  hour of day (0 = midnight)
N_t   = length(t_vec);

% Compute cos(θz) at each time step
cos_theta_z = zeros(1, N_t);
for i = 1:N_t
    t_h   = t_vec(i);
    h_rad = (t_h - 12) * 15 * pi/180;   % hour angle: 15°/h, negative AM
    ctz   = sin(phi_lat)*sin(delta_sun) + cos(phi_lat)*cos(delta_sun)*cos(h_rad);
    cos_theta_z(i) = max(0.0, ctz);     % zero when sun is below horizon
end

% Irradiance on horizontal surface at ground level
G_t = G_clearsky .* cos_theta_z;       % [W/m²] at each minute

% Solar power at panel output (before MPPT/charging)
A_solar      = 8 * (25 / (1000 * eta_solar_stc));  % [m²] physical area of 8× HL-25
%   Derivation: each 25W module rated at 1000 W/m², 11% efficiency
%   Area_per_module = 25 / (1000 × 0.11) = 0.2273 m² → 8 modules = 1.818 m²
%   Previous v1/v2 used 2.0 m² — no physical basis, now corrected

P_solar_panel  = eta_solar_eff * tilt_factor * A_solar .* G_t;  % [W] at panel terminal
P_solar_stored = P_solar_panel * eta_MPPT * eta_charge;          % [W] into battery

% Integrate solar energy using trapezoidal rule
E_solar_stored = trapz(t_vec, P_solar_stored);  % [Wh]

% Analytical check (for sine model: E = η_sys × A × G_peak × T/π)
% For cosine model: E = η_sys × A × G_clearsky × ∫cos(θz)dt
% (no simple closed form for non-equatorial case — use numerical result as ground truth)
P_solar_peak = eta_solar_eff * tilt_factor * A_solar * G_clearsky;

% Solar window (sunrise to sunset)
t_sunrise = t_vec(find(G_t > 0.5, 1, 'first'));
t_sunset  = t_vec(find(G_t > 0.5, 1, 'last'));

fprintf('[8] Solar Array — Astronomical Cosine Model [C3 — changed from sine]\n');
fprintf('    8× Ascent Solar HL-25 HyperLight CIGS 25W\n');
fprintf('    Physical panel area (8×0.2273 m²)    : %.4f m²  [C6 — was 2.0]\n', A_solar);
fprintf('    G_clearsky peak (Delhi summer, IMD)  : %.0f W/m²  [was 1000, now 900]\n', G_clearsky);
fprintf('    Astronomical sunrise                 : ~%.2fh (~%02.0f:%02.0f)\n', ...
        t_sunrise, floor(t_sunrise), (t_sunrise-floor(t_sunrise))*60);
fprintf('    Astronomical sunset                  : ~%.2fh (~%02.0f:%02.0f)\n', ...
        t_sunset, floor(t_sunset), (t_sunset-floor(t_sunset))*60);
fprintf('    Solar window (actual)                : %.1f h  [was 12h by assumption]\n', t_sunset-t_sunrise);
fprintf('    Peak panel output (panel terminal)   : %.1f W\n', P_solar_peak);
fprintf('    Peak power stored in battery         : %.1f W\n', P_solar_peak*eta_MPPT*eta_charge);
fprintf('    Total solar energy stored/day        : %.1f Wh  (numerical integral)\n\n', E_solar_stored);

%% =========================================================================
%  SECTION 9: PRE-FZERO POWER BUDGET
%  Powers that do NOT depend on hull volume are defined here.
%  P_cruise_elec requires V → computed AFTER fzero in Section 11.
% =========================================================================

% Avionics power — component-by-component:
%   Pixhawk 6C Mini:                2.5W  (STM32H743, all sensors active)
%   Raspberry Pi 4B (YOLOv8)      : 7.0W  (60% CPU load, RPi4 datasheet)
%   RPi AI Camera IMX500           : 1.5W  (on-sensor inference, RPi spec)
%   Holybro M10 GPS                : 0.5W  (uBlox M10 at 5Hz update rate)
%   SiK Telemetry v3 433MHz        : 1.0W  (1W TX at ~30% duty, receive 0.3W)
%   4× Hitec HS-65HB servos        : 2.8W  (0.7W each at partial duty, hitecrcd.com)
%   Misc (LEDs, voltage regulators): 0.7W
%   TOTAL:                         16.0W
P_avionics = 16.0;   % [W]

% Night standby — RPi4 in low-power mode (critical for battery check)
%   Pixhawk minimal (sensors off): 1.0W
%   RPi4 low-power idle           : 1.5W  (RPi4 documented idle consumption)
%   GPS acquisition mode          : 0.5W
%   Telemetry receive-only        : 0.3W
%   TOTAL:                         3.3W
P_standby = 3.3;     % [W] — significant: over 12h night = 39.6 Wh

% Winch motor electrical power:
%   Mechanical power: F × v_rope = (m_payload × g) × v_rope
%   v_rope = 0.1 m/s (controlled descent, safe for payload)
%   P_mech = 2.0 × 9.81 × 0.1 = 1.962 W
%   P_elec = P_mech / eta_winch = 1.962 / 0.533 = 3.68 W
%   Add holding torque current (between lower and release): +2.0 W
%   Rounded up: 6.0 W ← consistent with Pololu 37D max draw at 12V = 12W
P_winch = 6.0;       % [W]

% Ballonet fan:
P_fan = P_ballonet_fan;   % 15.0 W (Micronel U50L rated)

% Vertical motor (RL buoyancy shock):
%   Station-keeping: light downward force only, ~8W electrical
%   Shock arrest peak: MN3110 on 4S max = ~60W. 40W = 67% throttle → safe.
P_vert_hold = 8.0;   % [W] station-keeping
P_vert_peak = 40.0;  % [W] buoyancy shock arrest burst (30s duration)

fprintf('[9] Pre-fzero Power Budget\n');
fprintf('    P_avionics (full mission)         : %.1f W\n', P_avionics);
fprintf('    P_standby  (night, RPi low-power) : %.1f W\n', P_standby);
fprintf('    P_winch    (Pololu 37D)           : %.1f W\n', P_winch);
fprintf('    P_fan      (ballonet, Micronel)   : %.1f W\n', P_fan);
fprintf('    P_vert_hold (station-keep)        : %.1f W\n', P_vert_hold);
fprintf('    P_vert_peak (RL shock burst, 30s) : %.1f W\n', P_vert_peak);
fprintf('    P_cruise_elec → computed AFTER fzero (Section 11)\n\n');

%% =========================================================================
%  SECTION 10: HULL VOLUME SIZING — fzero MASS BALANCE
%
%  Mass balance at launch (worst case: empty ballonet, full payload loaded):
%    Buoyancy  = delta_rho × V_hull × g
%    Weight    = (m_envelope + M_hw + m_payload) × g
%    Ballonet is EMPTY at launch → full hull volume is helium
%
%  m_envelope = sigma_env × S(V_hull)   where S is the prolate spheroid
%                                        surface area function below
%  Solve: delta_rho × V - sigma_env × S(V) - M_hw - m_payload = 0
%
%  Hull volume V_hull = V_He_at_launch (ballonet empty, all He)
%  The physical hull must ALSO contain the ballonet at max inflation.
%  V_physical = V_He + V_ballonet_max (computed after solving for V_He)
%
%  This fzero is iterated twice because M_hw contains a ballonet bladder
%  mass term that depends on V_hull. The iteration converges in 2 passes.
% =========================================================================

% PASS 1: estimate hull volume with preliminary ballonet mass
mass_bal_fn = @(V) delta_rho * V ...
                   - sigma_env * hull_surface(V, k) ...
                   - M_hw - m_payload;

fz_opts = optimset('TolX', 1e-8, 'Display', 'off');
[V_He, ~, exitflag] = fzero(mass_bal_fn, 8.0, fz_opts);
if exitflag ~= 1
    error('fzero pass 1 did not converge — check input parameters');
end

% PASS 2: compute ballonet size from solved hull volume, update M_hw, re-solve
V_ballonet_actual = V_ballonet_frac * V_He;   % 25% of He volume
r_bal_actual = (3*V_ballonet_actual/(4*pi))^(1/3);
S_bal_actual = 4*pi*r_bal_actual^2;
m_bladder_actual = 0.035 * S_bal_actual;

% Replace preliminary bladder mass with actual value
M_hw_corrected = M_hw - m_bladder + m_bladder_actual;

mass_bal_fn2 = @(V) delta_rho * V ...
                    - sigma_env * hull_surface(V, k) ...
                    - M_hw_corrected - m_payload;
[V_He, ~, exitflag2] = fzero(mass_bal_fn2, V_He, fz_opts);
if exitflag2 ~= 1
    error('fzero pass 2 did not converge');
end
M_hw = M_hw_corrected;   % commit corrected M_hw

% Geometry from equilibrium volume
b_eq  = (3 * V_He / (4*pi*k))^(1/3);    % [m] semi-minor axis (max radius)
a_eq  = k * b_eq;                         % [m] semi-major axis (half-length)
D_eq  = 2 * b_eq;                         % [m] max diameter
L_eq  = 2 * a_eq;                         % [m] total length
S_env = hull_surface(V_He, k);            % [m²] envelope surface area

% Ballonet at actual hull size
V_ballonet = V_ballonet_frac * V_He;      % [m³] ballonet max volume
V_physical  = V_He + V_ballonet;          % [m³] total hull to manufacture

% Fill time per delivery
t_fill_min = V_ballonet / V_fan_flow;     % [min] per delivery cycle
m_air_full  = V_ballonet * rho_air_op;   % [kg] air mass when ballonet full

% Mass at launch
m_env    = sigma_env * S_env;
m_total  = m_env + M_hw + m_payload;

% Verify mass balance
F_buoy   = delta_rho * V_He * g;
F_weight = m_total * g;
residual = abs(F_buoy - F_weight);

% Check: does ballonet compensate the payload drop?
% After one 2kg drop + ballonet full: net mass change = –m_payload + m_air_full
mass_compensated = m_air_full - m_payload;

fprintf('[10] Volume Sizing (fzero, 2-pass iteration)\n');
fprintf('    ─────────────────────────────────────────\n');
fprintf('    Equilibrium He volume  V_He     : %.4f m³\n', V_He);
fprintf('    Ballonet max volume (25%% He)    : %.4f m³\n', V_ballonet);
fprintf('    Physical hull to manufacture     : %.4f m³\n', V_physical);
fprintf('    ─────────────────────────────────────────\n');
fprintf('    Hull length      L               : %.4f m\n', L_eq);
fprintf('    Hull diameter    D               : %.4f m\n', D_eq);
fprintf('    Semi-major axis  a               : %.4f m\n', a_eq);
fprintf('    Semi-minor axis  b               : %.4f m\n', b_eq);
fprintf('    Envelope surface S               : %.4f m²\n', S_env);
fprintf('    ─────────────────────────────────────────\n');
fprintf('    Envelope mass    m_env           : %.4f kg\n', m_env);
fprintf('    Hardware mass    M_hw (corrected): %.4f kg\n', M_hw);
fprintf('    Payload mass     m_payload        : %.4f kg\n', m_payload);
fprintf('    TOTAL at launch                  : %.4f kg\n', m_total);
fprintf('    ─────────────────────────────────────────\n');
fprintf('    Buoyancy force                   : %.4f N\n', F_buoy);
fprintf('    Weight force                     : %.4f N\n', F_weight);
fprintf('    Balance residual                 : %.2e N  (target: < 1e-4)\n', residual);
if residual > 1e-3
    fprintf('    WARNING: Balance residual too large — check parameters\n');
end
fprintf('    ─────────────────────────────────────────\n');
fprintf('    Ballonet fill time per delivery  : %.2f min\n', t_fill_min);
fprintf('    Air mass in full ballonet        : %.3f kg\n', m_air_full);
fprintf('    Mass compensation (air – payload): %+.3f kg  ', mass_compensated);
if abs(mass_compensated) < 0.3
    fprintf('(balanced within 15%% ✓)\n');
else
    fprintf('(significant imbalance — note in mission plan)\n');
end
fprintf('\n');

%% =========================================================================
%  SECTION 11: AERODYNAMIC DRAG & PROPULSION POWER
%
%  Two scenarios:
%  (A) Calm air, v = v_cruise  → used for energy budget
%  (B) Headwind, v = v_eff     → used for motor sizing (must handle this)
%
%  Drag equation (AIAA LTA standard, volumetric form):
%    D = 0.5 × ρ_air × v² × V^(2/3) × CDv
%  where V is the helium volume (proxy for body volume) and
%  CDv is referenced to V^(2/3) (not frontal area).
% =========================================================================

D_calm = 0.5 * rho_air_op * v_cruise^2 * V_He^(2/3) * CDv;   % [N] energy case
D_WC   = 0.5 * rho_air_op * v_eff^2   * V_He^(2/3) * CDv;   % [N] headwind case

% Electrical power to both rear cruise motors:
%   P_elec = (D × v_airspeed) / eta_drivetrain
P_cruise_elec    = (D_calm * v_cruise) / eta_drive;   % [W] calm — energy budget
P_cruise_elec_WC = (D_WC   * v_eff)   / eta_drive;   % [W] headwind — motor sizing

P_per_motor      = P_cruise_elec / 2;      % [W] per rear motor, energy budget
P_per_motor_WC   = P_cruise_elec_WC / 2;  % [W] per rear motor, worst case

% MN3110 KV470 on 4S max continuous = ~60W electrical per motor
headroom_energy = (1 - P_per_motor/60)    * 100;
headroom_WC     = (1 - P_per_motor_WC/60) * 100;

fprintf('[11] Aerodynamic Drag & Propulsion Power\n');
fprintf('    --- Energy Budget (calm air, v = %.1f m/s) ---\n', v_cruise);
fprintf('    Drag force D_calm              : %.4f N\n', D_calm);
fprintf('    Mechanical thrust power        : %.4f W\n', D_calm * v_cruise);
fprintf('    Total cruise electrical        : %.4f W  (both rear motors)\n', P_cruise_elec);
fprintf('    Per rear motor                 : %.4f W\n', P_per_motor);
fprintf('    Motor headroom (max 60W each)  : %.1f%%\n', headroom_energy);
fprintf('    --- Motor Sizing (headwind, v = %.1f m/s) ---\n', v_eff);
fprintf('    Drag force D_WC                : %.4f N\n', D_WC);
fprintf('    Total cruise electrical (WC)   : %.4f W\n', P_cruise_elec_WC);
fprintf('    Per rear motor (WC)            : %.4f W\n', P_per_motor_WC);
fprintf('    Motor headroom (WC)            : %.1f%%', headroom_WC);
if headroom_WC < 10
    fprintf('  ← WARNING: headroom < 10%% — consider MN4010 KV475 (120W max)\n');
elseif headroom_WC < 0
    fprintf('  ← CRITICAL: motors INSUFFICIENT for headwind!\n');
else
    fprintf('  ✓\n');
end
fprintf('\n');

%% =========================================================================
%  SECTION 12: COMPLETE MISSION POWER & ENERGY BUDGET
%
%  MISSION PROFILE (24-hour cycle):
%   00:00–06:00  (6h):  Night standby — moored, RPi low-power, Pixhawk minimal
%   06:00–14:00  (8h):  Transit cruise to delivery area
%   14:00–16:00  (2h):  4× delivery sequences (30 min each):
%                         • 5 min winch descent + release
%                         • 30s RL buoyancy shock arrest
%                         • ~4 min ballonet fill (fan)
%                         • Remainder: hover / station-keep
%   16:00–18:00  (2h):  Return cruise
%   18:00–00:00  (6h):  Night standby (second half)
%   Total cruise:  8 + 2 = 10h  | Total night: 6+6 = 12h
%
%  NOTE: Solar generation continues until ~19:00 (astronomical model).
%        The first hour of return cruise (16:00–17:00) is still solar-powered.
% =========================================================================

% Mission durations [hours]
t_cruise_h   = 10.0;                            % transit (8h out + 2h return)
t_hover_h    = n_deliver * 0.5;                 % 4 × 30 min hover = 2h
t_shock_h    = n_deliver * 30 / 3600;           % 4 × 30s = 0.0333h
t_winch_h    = n_deliver * 5 / 60;              % 4 × 5 min = 0.333h
t_fan_h      = n_deliver * t_fill_min / 60;    % 4 × t_fill_min = variable
t_night_h    = 12.0;                            % standby (midnight–6am, 6pm–midnight)

% Power in each mode [W]
P_mode_cruise  = P_cruise_elec + P_avionics;   % cruise motors + all avionics
P_mode_hover   = P_vert_hold   + P_avionics;   % vert motor (light) + avionics
P_mode_shock   = P_vert_peak   + P_avionics;   % vert motor (peak) + avionics
P_mode_winch   = P_winch       + P_avionics;   % winch + avionics (cruise off)
P_mode_fan     = P_fan         + P_avionics;   % ballonet fan + avionics
P_mode_night   = P_standby;                    % night standby only

% Energy per mode [Wh]
E_cruise  = P_mode_cruise * t_cruise_h;
E_hover   = P_mode_hover  * t_hover_h;
E_shock   = P_mode_shock  * t_shock_h;
E_winch   = P_mode_winch  * t_winch_h;
E_fan     = P_mode_fan    * t_fan_h;
E_night   = P_mode_night  * t_night_h;

E_total   = E_cruise + E_hover + E_shock + E_winch + E_fan + E_night;

fprintf('[12] Mission Power & Energy Budget\n');
fprintf('    %-34s  %9s  %8s  %10s\n', 'Mode', 'Power (W)', 'Time (h)', 'Energy (Wh)');
fprintf('    %-34s  %9.2f  %8.2f  %10.1f\n', 'Transit cruise (2× rear motors)', P_mode_cruise, t_cruise_h, E_cruise);
fprintf('    %-34s  %9.2f  %8.2f  %10.1f\n', 'Hover / station-keep (vert)', P_mode_hover, t_hover_h, E_hover);
fprintf('    %-34s  %9.2f  %8.4f  %10.2f\n', 'Buoyancy shock burst ×4', P_mode_shock, t_shock_h, E_shock);
fprintf('    %-34s  %9.2f  %8.3f  %10.1f\n', 'Winch operation ×4 (5 min ea)', P_mode_winch, t_winch_h, E_winch);
fprintf('    %-34s  %9.2f  %8.3f  %10.1f\n', 'Ballonet fan ×4 (vent+refill)', P_mode_fan, t_fan_h, E_fan);
fprintf('    %-34s  %9.2f  %8.2f  %10.1f\n', 'Night standby (RPi low-power)', P_mode_night, t_night_h, E_night);
fprintf('    %s\n', repmat('─', 1, 75));
fprintf('    %-34s  %9s  %8s  %10.1f\n', 'TOTAL CONSUMED', '', '', E_total);
fprintf('\n');

%% =========================================================================
%  SECTION 13: BATTERY STATE-OF-CHARGE ANALYSIS
% =========================================================================

Bat_rated_Wh  = 148.0;                       % [Wh] Tattu 4S 10000mAh (14.8V × 10Ah)
Bat_usable_Wh = Bat_rated_Wh * DoD_limit;    % [Wh] 80% DoD = 118.4 Wh
Bat_reserve_Wh = Bat_rated_Wh * (1 - DoD_limit);  % [Wh] 20% SoC floor

% Night standby uses battery (no solar at night)
night_margin = Bat_usable_Wh - E_night;

fprintf('[13] Battery Analysis — Tattu 4S 10000mAh LiPo\n');
fprintf('    Rated capacity                : %.1f Wh (at 14.8V, 10Ah)\n', Bat_rated_Wh);
fprintf('    Usable capacity (80%% DoD)    : %.1f Wh\n', Bat_usable_Wh);
fprintf('    20%% SoC reserve (hard floor) : %.1f Wh  ← never discharge below this\n', Bat_reserve_Wh);
fprintf('    Night standby demand (12h)   : %.1f Wh\n', E_night);
fprintf('    Night battery margin          : %.1f Wh  (%.0f%% of usable)\n', ...
        night_margin, night_margin/Bat_usable_Wh*100);
if night_margin < 10
    fprintf('    WARNING: night margin < 10 Wh — battery may not sustain 12h night.\n');
    fprintf('    Consider Tattu 4S 12000mAh (178 Wh, 720g) or add second battery.\n');
elseif night_margin < 0
    fprintf('    CRITICAL: Battery insufficient for night standby. Upgrade required.\n');
else
    fprintf('    Night buffer: ADEQUATE ✓\n');
end
fprintf('\n');

%% =========================================================================
%  SECTION 14: NET ENERGY BALANCE — NET-ZERO VERIFICATION
%
%  The central claim of this project is that the airship achieves
%  net-zero energy over a 24-hour diurnal cycle.
%  This section verifies it quantitatively.
%
%  INTERPRETATION:
%  E_solar_stored is the energy deposited into the battery by the solar array.
%  E_total is the energy drawn from the battery by all loads.
%  If E_solar_stored ≥ E_total: the battery state-of-charge at end of day
%  is ≥ SoC at start of day → mission is indefinitely sustainable → net-zero ✓
% =========================================================================

E_net = E_solar_stored - E_total;

fprintf('[14] Net Energy Balance — Net-Zero Verification\n');
fprintf('    Solar energy stored (24h)     : %8.1f Wh\n', E_solar_stored);
fprintf('    Total mission energy consumed : %8.1f Wh\n', E_total);
fprintf('    NET BALANCE                   : %8.1f Wh\n', E_net);
if E_net >= 0
    fprintf('    STATUS: NET-POSITIVE ✓  (+%.1f Wh = +%.0f%% margin above consumption)\n', ...
            E_net, E_net/E_total*100);
    fprintf('    Net-zero operational claim is VALIDATED.\n');
else
    fprintf('    STATUS: ENERGY DEFICIT ✗  — mission is NOT net-zero as configured.\n');
    A_extra = abs(E_net) / (eta_solar_system * G_clearsky * (t_sunset-t_sunrise) * 2/pi);
    fprintf('    Additional solar area needed: %.2f m² (total: %.2f m²)\n', ...
            A_extra, A_solar + A_extra);
end
fprintf('\n');

%% =========================================================================
%  SECTION 15: BUOYANCY SHOCK — ALTITUDE EXCURSION (UNCONTROLLED)
%
%  If no RL motor or ballonet acts after payload drop:
%  System is m_payload = 2 kg lighter.
%  Airship rises until new equilibrium altitude where rho_air is lower.
%  ISA exponential atmosphere: rho(h) = rho0 × exp(–h/H)  where H = 8500m
% =========================================================================

H_atm     = 8500;   % [m] ISA atmospheric scale height
h_cruise  = 100;    % [m] assumed cruise/delivery altitude

% Air density at cruise altitude
rho_at_h  = rho_air_op * exp(-h_cruise / H_atm);

% New equilibrium: delta_rho_new × V_He = m_total – m_payload
% delta_rho_new = rho_new_air – rho_He_hot ≈ rho_new_air (He density changes negligibly)
% rho_new_air × V_He ≈ m_total – m_payload – m_env – M_hw + V_He × rho_He_hot
% Simplification: solve for rho_new_air needed
rho_needed  = (m_total - m_payload) / V_He + rho_He_hot;  % [kg/m³] needed air density
h_new       = -H_atm * log(rho_needed / rho_air_op);      % [m] new equilibrium altitude
delta_h     = h_new - h_cruise;                            % [m] altitude excursion

fprintf('[15] Buoyancy Shock — Uncontrolled Altitude Excursion\n');
fprintf('    Cruise altitude                        : %.0f m\n', h_cruise);
fprintf('    System mass at cruise (with payload)   : %.3f kg\n', m_total);
fprintf('    System mass after drop (no payload)    : %.3f kg\n', m_total - m_payload);
fprintf('    Air density at cruise altitude         : %.4f kg/m³\n', rho_at_h);
fprintf('    Air density needed for new balance     : %.4f kg/m³\n', rho_needed);
fprintf('    New passive equilibrium altitude       : %.0f m\n', h_new);
fprintf('    UNCONTROLLED ALTITUDE EXCURSION        : +%.0f m (+%.2f km)\n', delta_h, delta_h/1000);
fprintf('    → RL motor handles transient (0–30s): arrest climbing velocity.\n');
fprintf('    → Ballonet handles steady-state trim: add air mass to restore balance.\n');
fprintf('    → Without BOTH systems, airship climbs %.0f m — unacceptable.\n\n', delta_h);

%% =========================================================================
%  SECTION 16: RL AGENT — UPDATED STATE/ACTION SPACE
% =========================================================================

fprintf('[16] RL Agent State/Action Space\n');
fprintf('    STATE observations (8 continuous):\n');
fprintf('      s1: Vertical acceleration  [m/s²] — ICM-42688-P IMU (Pixhawk built-in)\n');
fprintf('      s2: Vertical velocity      [m/s]  — IMU integration (EKF output)\n');
fprintf('      s3: Altitude error         [m]    — MS5611 barometer (Pixhawk built-in)\n');
fprintf('      s4: Pitch angle            [deg]  — EKF fused estimate\n');
fprintf('      s5: Yaw angle              [deg]  — EKF fused estimate\n');
fprintf('      s6: Winch motor current    [A]    — Cytron MD13S driver feedback pin\n');
fprintf('      s7: Rope deployed length   [m]    — encoder on winch drum\n');
fprintf('      s8: Ballonet fill fraction [-]    — estimated from fan run-time × V_fan_flow\n');
fprintf('    ACTION outputs (4 continuous, normalized [-1, +1]):\n');
fprintf('      a1: Vertical motor PWM     — downward thrust force (main RL action)\n');
fprintf('      a2: Elevator deflection    — ±30° (pitch moment for nose-down assist)\n');
fprintf('      a3: Port motor delta       — differential throttle for yaw\n');
fprintf('      a4: Stbd motor delta       — differential throttle for yaw\n');
fprintf('    BALLONET control (PID altitude hold, NOT RL — different time scale):\n');
fprintf('      Fan ON/OFF: commanded by altitude error integral\n');
fprintf('      Timescale: minutes. RL handles: seconds (0–30s transient).\n');
fprintf('      Intake valve servo opens when filling; exhaust opens when venting.\n\n');

%% =========================================================================
%  SECTION 17: FINAL SUMMARY
% =========================================================================

fprintf('=================================================================\n');
fprintf(' FINAL SIZING SUMMARY — SURA AIRSHIP v3.0\n');
fprintf('=================================================================\n');
fprintf('  ── HULL ──────────────────────────────────────────────────────\n');
fprintf('  Shape (fineness ratio k)     : Prolate Spheroid, k = %.1f  [C1]\n', k);
fprintf('  CDv                          : %.3f  [C2 — verified Khoury Tab. 2.1]\n', CDv);
fprintf('  Equilibrium He volume        : %.4f m³\n', V_He);
fprintf('  Ballonet max volume (25%%)   : %.4f m³  [C4 — FIXED from v2]\n', V_ballonet);
fprintf('  Physical hull to build       : %.4f m³\n', V_physical);
fprintf('  Hull length L                : %.4f m\n', L_eq);
fprintf('  Hull max diameter D          : %.4f m\n', D_eq);
fprintf('  Hull surface area S          : %.4f m²\n', S_env);
fprintf('  ── MASS ──────────────────────────────────────────────────────\n');
fprintf('  Total launch mass            : %.4f kg\n', m_total);
fprintf('    Envelope                   : %.4f kg\n', m_env);
fprintf('    Hardware M_hw              : %.4f kg\n', M_hw);
fprintf('    Payload                    : %.4f kg\n', m_payload);
fprintf('  ── DRAG & PROPULSION ────────────────────────────────────────\n');
fprintf('  Drag at 5 m/s (calm)         : %.4f N  (energy budget)\n', D_calm);
fprintf('  Drag at 10 m/s (headwind)    : %.4f N  (motor sizing)\n', D_WC);
fprintf('  Cruise electrical (calm)     : %.3f W  (both rear motors)\n', P_cruise_elec);
fprintf('  Cruise electrical (headwind) : %.3f W\n', P_cruise_elec_WC);
fprintf('  ── SOLAR ─────────────────────────────────────────────────────\n');
fprintf('  Solar model                  : Astronomical cosine (lat/decl)  [C3]\n');
fprintf('  Panel area (8× HL-25)        : %.4f m²  [C6]\n', A_solar);
fprintf('  Effective solar efficiency   : %.2f%%\n', eta_solar_eff*100);
fprintf('  Solar energy stored/day      : %.1f Wh\n', E_solar_stored);
fprintf('  ── ENERGY ────────────────────────────────────────────────────\n');
fprintf('  Total consumed/day           : %.1f Wh\n', E_total);
fprintf('  NET BALANCE                  : %+.1f Wh  (%s)\n', E_net, ...
        char(string(ternary_str(E_net >= 0, 'NET-POSITIVE ✓', 'DEFICIT ✗'))));
fprintf('=================================================================\n\n');

%% =========================================================================
%  SECTION 18: SENSITIVITY ANALYSIS SWEEP
%  Runs parametric sweeps over k, solar efficiency, and payload mass
%  to show robustness of the design.
% =========================================================================

fprintf('[18] Running sensitivity analysis sweeps...\n');

% Sweep 1: fineness ratio k vs required He volume
k_range  = 3.0:0.1:5.5;
V_k      = zeros(size(k_range));
CDv_k    = zeros(size(k_range));
for i = 1:length(k_range)
    ki  = k_range(i);
    CDv_ki = 0.025 + 0.005 * abs(ki - 4.0);  % approximate parabolic CDv(k) near k=4
    fn  = @(V) delta_rho*V - sigma_env*hull_surface(V,ki) - M_hw - m_payload;
    try
        V_k(i) = fzero(fn, 8.0, fz_opts);
    catch
        V_k(i) = NaN;
    end
    CDv_k(i) = CDv_ki;
end

% Sweep 2: effective solar efficiency vs net energy
eta_range = 0.06:0.005:0.16;
E_net_eta = zeros(size(eta_range));
for i = 1:length(eta_range)
    P_sp = eta_range(i) * tilt_factor * eta_MPPT * eta_charge * A_solar .* G_t;
    E_net_eta(i) = trapz(t_vec, P_sp) - E_total;
end

% Sweep 3: payload mass vs required He volume
mpay_range = 0.5:0.1:4.0;
V_mpay     = zeros(size(mpay_range));
for i = 1:length(mpay_range)
    fn = @(V) delta_rho*V - sigma_env*hull_surface(V,k) - M_hw - mpay_range(i);
    try
        V_mpay(i) = fzero(fn, V_He, fz_opts);
    catch
        V_mpay(i) = NaN;
    end
end

fprintf('    Sweeps complete.\n\n');

%% =========================================================================
%  SECTION 19: PLOTS
% =========================================================================

fig1 = figure('Name','SURA Airship v3.0 — Results', ...
              'NumberTitle','off', 'Position',[50 50 1500 950]);

% --- Plot 1: Solar irradiance models (old vs new) ---
ax1 = subplot(2,3,1);
G_sine_old = zeros(1, N_t);
for i = 1:N_t
    ti = t_vec(i);
    if ti >= 6 && ti <= 18
        G_sine_old(i) = 1000 * sin(pi*(ti-6)/12);
    end
end
plot(t_vec, G_sine_old, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Old: G_0 sin(π(t-6)/12)'); hold on;
plot(t_vec, G_t, 'b-', 'LineWidth', 2, 'DisplayName', 'New: G_{clr}cos(θ_z) [astro]');
area(t_vec, G_t, 'FaceAlpha', 0.12, 'FaceColor', 'b', 'EdgeColor', 'none', 'DisplayName', '');
xline(t_sunrise, 'b:', 'LineWidth', 1.2);
xline(t_sunset,  'b:', 'LineWidth', 1.2);
xline(6,  'r:', 'LineWidth', 1.0);
xline(18, 'r:', 'LineWidth', 1.0);
ylabel('Irradiance  [W/m²]'); xlabel('Hour of day');
title({'Solar Irradiance Model Comparison', '[C3: sine → astro cosine]'});
legend('Old sine (v1/v2)', 'New cosine (v3)', 'Location', 'n'); grid on;
xlim([4 21]); xticks(4:2:21);
text(5.5, 820, sprintf('Old: E=%.0fWh/m²\nNew: E=%.0fWh/m²', ...
    trapz(t_vec,G_sine_old), trapz(t_vec,G_t)), 'FontSize',7, 'Color','k');

% --- Plot 2: Cumulative energy balance ---
ax2 = subplot(2,3,2);
E_cum_solar = cumtrapz(t_vec, P_solar_stored);
% Build a simple stepped power consumption profile for plotting
P_cons_t = build_power_profile(t_vec, P_mode_cruise, P_mode_hover, ...
           P_mode_fan, P_mode_night, 12.0);
E_cum_cons = cumtrapz(t_vec, P_cons_t);
plot(t_vec, E_cum_solar, 'g-', 'LineWidth', 2, 'DisplayName', 'Solar stored'); hold on;
plot(t_vec, E_cum_cons, 'r-', 'LineWidth', 2, 'DisplayName', 'Consumed');
surplus_mask = E_cum_solar > E_cum_cons;
fill([t_vec, fliplr(t_vec)], [E_cum_solar, fliplr(E_cum_cons)], ...
     'g', 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'DisplayName', 'Surplus');
xlabel('Hour of day'); ylabel('Cumulative energy  [Wh]');
title('Energy Balance Over 24 Hours'); xlim([0 24]); xticks(0:4:24);
legend('Location', 'nw'); grid on;
text(15, E_cum_solar(end)*0.5, sprintf('Net: %+.0f Wh', E_net), ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'Color', ternary_color(E_net>=0, [0 0.5 0], [0.8 0 0]));

% --- Plot 3: Energy by mission mode ---
ax3 = subplot(2,3,3);
mode_names = {'Cruise','Hover','Shock','Winch','Ballonet','Night'};
mode_E     = [E_cruise, E_hover, E_shock, E_winch, E_fan, E_night];
clrs = [0.20 0.40 0.80;  0.30 0.70 0.30;  0.90 0.30 0.30; ...
        0.80 0.60 0.10;  0.40 0.80 0.80;  0.55 0.55 0.55];
bh = bar(mode_E, 'FaceColor', 'flat');
bh.CData = clrs;
set(gca, 'XTickLabel', mode_names, 'FontSize', 9);
ylabel('Energy  [Wh]'); title('Energy Consumption by Mission Mode'); grid on;
for ii = 1:length(mode_E)
    text(ii, mode_E(ii) + 1, sprintf('%.1f', mode_E(ii)), ...
         'HorizontalAlignment', 'center', 'FontSize', 8);
end

% --- Plot 4: Mass budget pie ---
ax4 = subplot(2,3,4);
cat_names = {'Envelope','Propulsion','Control','Computers', ...
             'Navigation','Power','Payload Mech','Ballonet','Structural'};
cat_vals  = [m_env, m_propulsion, m_control, m_computers, ...
             m_navigation, m_power, m_payload_mech, m_ballonet, m_structural];
pie_filtered = cat_vals(cat_vals > 0.01);
name_filtered = cat_names(cat_vals > 0.01);
pie(pie_filtered);
legend(name_filtered, 'Location', 'bestoutside', 'FontSize', 6);
title(sprintf('Mass Budget (total = %.2f kg)', m_total));

% --- Plot 5: RL shock response timeline ---
ax5 = subplot(2,3,5);
t_ev = -20:0.1:200;
P_vert_t = zeros(size(t_ev));
for i = 1:length(t_ev)
    tt = t_ev(i);
    if tt >= -5 && tt < 0
        P_vert_t(i) = P_vert_hold + (P_vert_peak - P_vert_hold) * (tt+5)/5;
    elseif tt >= 0 && tt < 30
        P_vert_t(i) = P_vert_peak;
    elseif tt >= 30 && tt < 45
        P_vert_t(i) = P_vert_peak * (1 - (tt-30)/15);
    elseif tt >= 45
        P_vert_t(i) = P_vert_hold;
    else
        P_vert_t(i) = 0;
    end
end
plot(t_ev, P_vert_t, 'b-', 'LineWidth', 2, 'DisplayName', 'Vert motor (RL)'); hold on;
xline(0, 'r--', 'LineWidth', 2, 'Label', 'Drop', 'LabelVerticalAlignment', 'bottom');
xline(-5, 'g--', 'LineWidth', 1.2, 'Label', 'RL pre-arm', 'LabelVerticalAlignment', 'bottom');
yline(P_vert_hold, 'k:', 'LineWidth', 1, 'Label', 'Hover hold');
xlabel('Time from payload release  [s]');
ylabel('Vertical motor power  [W]');
title('RL Buoyancy Shock Response Timeline');
legend('Location', 'ne'); ylim([0 55]); xlim([-20 200]); grid on;

% --- Plot 6: Hull cross-section schematic ---
ax6 = subplot(2,3,6);
phi_hull = linspace(-pi/2, pi/2, 300);
x_hull = a_eq * cos(phi_hull);
y_hull = b_eq * sin(phi_hull);
fill([x_hull, fliplr(x_hull)], [y_hull, fliplr(-y_hull)], [0.75 0.88 1.0], ...
     'FaceAlpha', 0.5, 'EdgeColor', 'b', 'LineWidth', 2); hold on;
% Ballonet bladder (shown at rear interior)
r_b_draw = min(b_eq*0.55, (3*V_ballonet/(4*pi))^(1/3));
theta_b  = linspace(0, 2*pi, 100);
x_b = (a_eq*0.15) + r_b_draw*0.75*cos(theta_b);
y_b = r_b_draw*0.55*sin(theta_b);
fill(x_b, y_b, [1 0.7 0.3], 'FaceAlpha', 0.6, 'EdgeColor', [0.8 0.4 0], 'LineWidth', 1.5);
text(a_eq*0.15, -r_b_draw*0.65, sprintf('Ballonet\n%.2fm³',V_ballonet), ...
    'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', [0.5 0.2 0]);
% Solar panels on hull top
x_panels = linspace(-a_eq*0.5, a_eq*0.5, 14);
y_panels = arrayfun(@(x) b_eq*sqrt(max(0, 1-(x/a_eq)^2)), x_panels);
scatter(x_panels, y_panels, 45, [0.1 0.7 0.1], 'filled', 'DisplayName', 'Solar panels');
% Gondola
rectangle('Position', [-0.35, -(b_eq+0.30), 0.70, 0.22], ...
          'Curvature', [0.3 0.3], 'FaceColor', [0.55 0.38 0.18], 'EdgeColor', 'k');
text(0, -(b_eq+0.18), 'Gondola', 'HorizontalAlignment', 'center', 'FontSize', 7);
% Motor positions
scatter([a_eq*0.82, a_eq*0.82], [b_eq*0.30, -b_eq*0.30], 80, 'r', 'filled', 'DisplayName', 'Cruise motors');
text(a_eq*0.85, b_eq*0.42, 'Port', 'FontSize', 7);
text(a_eq*0.85, -b_eq*0.42, 'Stbd', 'FontSize', 7);
scatter(0, -(b_eq+0.05), 80, [0.45 0 0.75], 'filled', 'DisplayName', 'Vert motor');
text(0.1, -(b_eq+0.05), 'Vert', 'FontSize', 7, 'Color', [0.45 0 0.75]);
% Fan inlet marker
scatter(-a_eq*0.65, 0, 50, [0 0.6 0.6], 'filled', 'DisplayName', 'Fan inlet');
text(-a_eq*0.65+0.05, 0.1, 'Fan', 'FontSize', 7, 'Color', [0 0.5 0.5]);
axis equal; grid on;
xlabel('Longitudinal axis  [m]'); ylabel('Lateral axis  [m]');
title(sprintf('Hull Profile: L=%.2fm  D=%.2fm  V_{He}=%.2fm³', L_eq, D_eq, V_He));

sgtitle('SURA Airship v3.0 — Sizing & Simulation Results', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% =========================================================================
%  SECTION 20: SENSITIVITY ANALYSIS PLOTS
% =========================================================================

fig2 = figure('Name', 'SURA Airship v3.0 — Sensitivity Analysis', ...
              'NumberTitle', 'off', 'Position', [100 100 1300 450]);

subplot(1,3,1);
plot(k_range, V_k, 'b-o', 'LineWidth', 2, 'MarkerSize', 4);
xline(k, 'r--', sprintf('k=%.1f (chosen)', k), 'LineWidth', 1.5, ...
      'LabelVerticalAlignment', 'bottom');
xlabel('Fineness ratio k'); ylabel('Required He volume  [m³]');
title({'k vs Equilibrium Volume', '(CDv approximated parabolic near k=4)'}); grid on;

subplot(1,3,2);
plot(eta_range*100, E_net_eta, 'g-o', 'LineWidth', 2, 'MarkerSize', 4);
xline(eta_solar_eff*100, 'r--', sprintf('η=%.1f%% (design)', eta_solar_eff*100), ...
      'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
yline(0, 'k-', 'Net-zero threshold', 'LineWidth', 1.5);
xlabel('Effective solar efficiency  [%]'); ylabel('Net energy balance  [Wh]');
title('Solar Efficiency Sensitivity'); grid on;

subplot(1,3,3);
plot(mpay_range, V_mpay, 'm-o', 'LineWidth', 2, 'MarkerSize', 4);
xline(m_payload, 'r--', sprintf('m_{pay}=%.0fkg (design)', m_payload), ...
      'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
xlabel('Payload mass  [kg]'); ylabel('Required He volume  [m³]');
title('Payload Mass vs Required Volume'); grid on;

sgtitle('SURA Airship v3.0 — Sensitivity Analysis', 'FontSize', 13, 'FontWeight', 'bold');

fprintf('[20] All figures generated. Simulation complete.\n');
fprintf('=================================================================\n');
fprintf(' v3.0 COMPLETE — Agya Sanghi & Kartik Aggarwal | SURA 2025\n');
fprintf('=================================================================\n');

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function S = hull_surface(V, k)
    % Prolate spheroid surface area (exact formula via arc-sin eccentricity)
    % V = (4/3)π a b² with a = k·b → b = (3V/(4πk))^(1/3)
    % S = 2πb²(1 + (a/b)·arcsin(e)/e) where e = √(1-(b/a)²)
    b = (3*V / (4*pi*k))^(1/3);
    a = k * b;
    e = sqrt(1 - (b/a)^2);
    S = 2*pi*b^2 * (1 + (a/b)*asin(e)/e);
end

function s = ternary_str(cond, s_true, s_false)
    if cond; s = s_true; else; s = s_false; end
end

function c = ternary_color(cond, c_true, c_false)
    if cond; c = c_true; else; c = c_false; end
end

function P_t = build_power_profile(t_vec, P_cruise, P_hover, P_inflate, P_night, t_night_h)
    % Simplified power profile for cumulative energy plot
    % (Not used in energy budget calculation — that uses mode × duration)
    P_t = zeros(size(t_vec));
    t_day_end = 24 - t_night_h/2;
    for i = 1:length(t_vec)
        ti = t_vec(i);
        if ti < 6 || ti >= t_day_end
            P_t(i) = P_night;
        elseif ti >= 14 && ti <= 16
            P_t(i) = P_hover + P_inflate*0.5;
        else
            P_t(i) = P_cruise;
        end
    end
end