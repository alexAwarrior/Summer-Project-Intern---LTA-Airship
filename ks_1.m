M_payload = 2.0;       % given load
M_hw = 2.0;            % estimated mass of other components required
rho_air = 1.225;       
rho_he = 0.1786;       
delta_rho = rho_air - rho_he; 
sigma_env = 0.1;       % heavy duty mylar was around 0.07 kg/m2 but 
% rounded up higher to account for error
k = 3.5;               % length / diameter, for blimps this ratio is 
% between 3 to 5
v_cruise = 5.0;        % target cruise speed (m/s) (~18 km/h)
C_dv = 0.03;           % for airships, volumetric drag coefficient is
%  taken, for blimps it lies between 0.023 and 0.045
eta_prop = 0.8;        % propeller efficiency taken to be 80%
eta_motor = 0.8;       % motor efficiency taken to be 80%
P = 10;       % we assume 10W of power is drawn continuously by the other
%  components with majority power drawn by say arduino or raspberry pi 4


%all equations used have been referred to from the internet and thus may
%hold their errors
ecc = sqrt(1 - 1/k^2);
mass_eq = @(D) (pi/6 * k * D.^3 * delta_rho) - (M_payload + M_hw + (2*pi*(D/2).^2 * (1 + (k/ecc)*asin(ecc))) * sigma_env);
%we use the surface area of an ellipse rotated along its major axis
D_req = fzero(mass_eq, 1.5);
L_req = k * D_req;
V_req = (pi/6) * k * D_req^3;
S_req = 2*pi*(D_req/2)^2 * (1 + (k/ecc)*asin(ecc));
M_env = S_req * sigma_env;
M_total = M_payload + M_hw + M_env;

Drag = 0.5 * rho_air * v_cruise^2 * (V_req^(2/3)) * C_dv; %standard drag formula
P_thrust = (Drag * v_cruise) / (eta_prop * eta_motor); 
%we could assume 100% efficient but that would not be practical
P_total = P_thrust + P;
%displaying stuff (co-pilot)
fprintf('Required Diameter: %.2f m\n', D_req);
fprintf('Required Length:   %.2f m\n', L_req);
fprintf('Buoyant Volume:    %.2f m^3\n', V_req);
fprintf('Total Mass:        %.2f kg (Payload: %.1f, HW: %.1f, Env: %.2f)\n', M_total, M_payload, M_hw, M_env);
fprintf('Aerodynamic Drag:  %.2f N @ %.1f m/s\n', Drag, v_cruise);
fprintf('Total Power Req:   %.2f W (%.2f W Thrust + %.2f W Avionics)\n', P_total, P_thrust, P);

time_hours = linspace(6, 18, 1000); % 6 AM to 6 PM
dt = (time_hours(2) - time_hours(1)) * 3600; 

P_cruise = 15;      % we did calculate 11.35 W but now that made in still 
% air so now the power requirement is bumped up to account for disturbances
P_hover = 5;       % required during hovering/package drop (no drag)
P_avionics = 10;
Bat_Capacity_Wh = 150; % common consumer drones have a 100 Wh battery and 
% LiPo batteries can be used which fit in the weight limit as well
%https://www.sciencedirect.com/science/article/pii/S0038092X24006303
Bat_Energy_J = Bat_Capacity_Wh * 3600; 

A_solar = 2.0;      % 2 square meters of thin film panels on top, 
% taken as 2 square meters to stay within the proposed weight limit
eta_solar = 0.15;   % 15% efficiency for ultra thin film CIGS solar cells
Peak_Irradiance = 1000; % taken from net, commonly used for simulating


flight_mode = ones(size(time_hours)); % we assume 1 to mean flight (transit)
% Insert Hovering phases (e.g., at 8:00, 11:00, 14:00, 16:00 for 30 mins each)
delivery_times = [8, 11, 14, 16]; %we take 4 delivery runs
for t = delivery_times
    idx = (time_hours >= t) & (time_hours <= t+0.5); %we take a 30 minute 
    % window because, there are a lot of things that can go wrong with a 
    % large blimp, it has to flight at a sufficient high and has to deal
    % with a lot of extra thrust when the load is dropped
    flight_mode(idx) = 0; 
end

Power_Consumed = (flight_mode .* P_cruise) + (~flight_mode .* P_hover) + P_avionics;

Irradiance = Peak_Irradiance * sin(pi * (time_hours - 6) / 12); %irradiance
% modelled as a sine wave
Irradiance(Irradiance < 0) = 0;

Power_Generated = Irradiance * A_solar * eta_solar;

plot(time_hours, Power_Generated, 'g', 'LineWidth', 2);
hold on;
plot(time_hours, Power_Consumed, 'r', 'LineWidth', 2);
title('Power Budget: Generation vs Consumption');
xlabel('Time of Day (24h)');
ylabel('Power (Watts)');
grid on; 
xlim([6 18]);

