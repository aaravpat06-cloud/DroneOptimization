clear; clc; close all;

%% =========================================================================
% 1. USER INPUT CONFIGURATION (CHANGE THESE VALUES TO ITERATE YOUR DESIGN)
% =========================================================================

% --- Frame & Mass Inputs ---
arm_len       = 0.015;  % Length of the drone arm (meters)
hub_mass      = 0.035;  % Mass of the central hub (kg)
battery_mass  = 0.200;  % Mass of the battery (kg)
motor_mass    = 0.035;  % Mass of a single motor (kg)

% --- Thrust & Propulsion Inputs ---
max_thrust_per_motor = 4.5;   % Max thrust from one motor (Newtons)
target_TWR           = 2.5;   % Target Thrust-to-Weight Ratio

% --- Environment & Aero Inputs ---
wind_speed = 12.0;      % Simulated wind gust (m/s)
rho        = 1.225;     % Air density (kg/m^3)
Cd         = 1.3;       % Drag coefficient
Area       = 0.015;     % Projected frontal area (m^2)

% --- PID Control Inputs ---
Kp = 2.0;               % Proportional gain
Kd = 0.5;               % Derivative gain

% --- Structural & Material Inputs (Carbon Fiber- alt. to PLA) ---
E_modulus    = 150e9;   % Young's Modulus (Pascals)
outer_radius = 0.008;   % Arm outer radius (meters)
inner_radius = 0.007;   % Arm inner radius (meters)
arm_mass     = 0.015;   % Mass of the carbon tube itself (kg)

% --- Flight Envelope Inputs ---
min_flight_rpm = 5000;  % Hover RPM
max_flight_rpm = 25000; % Max throttle RPM


%% =========================================================================
% 2. COMPONENT SETUP & GEOMETRY
% =========================================================================
m_pos = arm_len * cos(pi/4);

% Hub
components(1).name = 'central hub';
components(1).mass = hub_mass;
components(1).pos  = [0.0, 0.0, 0.0];

% Battery
components(2).name = 'Battery';
components(2).mass = battery_mass;
components(2).pos  = [0.0, 0.0, -0.02];

% Motors
components(3).name = 'Motor Front-Right';
components(3).mass = motor_mass;
components(3).pos  = [m_pos, -m_pos, 0.01];

components(4).name = 'Motor Front-Left';
components(4).mass = motor_mass;
components(4).pos  = [m_pos, m_pos, 0.01];

components(5).name = 'Motor Back-Left';
components(5).mass = motor_mass;
components(5).pos  = [-m_pos, m_pos, 0.01];

components(6).name = 'Motor Back-Right';
components(6).mass = motor_mass;
components(6).pos  = [-m_pos, -m_pos, 0.01];


%% =========================================================================
% 3. MASS & INERTIA CALCULATIONS
% =========================================================================
total_mass = 0;
weighted_pos_sum = [0, 0, 0];
numComponents = length(components);

% Calculate total mass and weighted positions
for i = 1:numComponents
    m = components(i).mass;
    p = components(i).pos;
    total_mass = total_mass + m;
    weighted_pos_sum = weighted_pos_sum + (m * p);
end

% Calculate Center of Gravity (CG)
CG = weighted_pos_sum / total_mass;

% Calculate Moments of Inertia
I_xx = 0; I_yy = 0; I_zz = 0;
for i = 1:numComponents
    m = components(i).mass;
    rel_pos = components(i).pos - CG;
    x = rel_pos(1);
    y = rel_pos(2);
    z = rel_pos(3);
   
    I_xx = I_xx + m * (y^2 + z^2);
    I_yy = I_yy + m * (x^2 + z^2);
    I_zz = I_zz + m * (x^2 + y^2);
end

fprintf('\n================ RESULTS ================\n');
fprintf('Total Drone Mass: %.3f kg (%.1f g)\n', total_mass, total_mass * 1000);
fprintf('Center of Gravity (CG) relative to Hub center:\n');
fprintf('  X (Forward/Back): %.4f m\n', CG(1));
fprintf('  Y (Left/Right):   %.4f m\n', CG(2));
fprintf('  Z (Up/Down):      %.4f m\n', CG(3));
fprintf('-----------------------------------------\n');
fprintf('Moments of Inertia:\n');
fprintf('  I_xx (Roll):  %.6f kg*m^2\n', I_xx);
fprintf('  I_yy (Pitch): %.6f kg*m^2\n', I_yy);
fprintf('  I_zz (Yaw):   %.6f kg*m^2\n', I_zz);
fprintf('=========================================\n');


%% =========================================================================
% 4. THRUST CALCULATIONS
% =========================================================================
g = 9.81; % Gravity acceleration (m/s^2)
total_max_thrust = max_thrust_per_motor * 4;
drone_weight = total_mass * g; 

required_total_hover_thrust = drone_weight;
required_motor_hover_thrust = required_total_hover_thrust / 4;
required_total_max_thrust = drone_weight * target_TWR;
required_motor_max_thrust = required_total_max_thrust / 4;

actual_TWR = total_max_thrust / drone_weight;

fprintf('\n================ THRUST ANALYSIS ================\n');
fprintf('Drone Weight: %.2f N\n', drone_weight);
fprintf('Target TWR:   %.1f\n', target_TWR);
fprintf('Actual TWR:   %.2f\n', actual_TWR);
fprintf('-------------------------------------------------\n');
fprintf('Per-Motor Thrust Required to Hover: %.2f N\n', required_motor_hover_thrust);
fprintf('Per-Motor Thrust Required for Max:   %.2f N\n', required_motor_max_thrust);
fprintf('Selected Motor Max Capability:       %.2f N\n', max_thrust_per_motor);
fprintf('-------------------------------------------------\n');

if actual_TWR < 1.5
    fprintf('CRITICAL STATUS: UNDERPOWERED!\n');
    fprintf('Your drone might not even lift off safely. Decrease mass or find stronger motors.\n');
elseif actual_TWR < target_TWR
    fprintf('WARNING STATUS: Marginal Performance.\n');
    fprintf('The drone will fly, but it does not meet your target TWR of %.1f.\n', target_TWR);
else
    fprintf('STATUS: DESIGN PASSED!\n');
    fprintf('Your selected motors provide excellent power for this frame weight.\n');
end
fprintf('=================================================\n');


%% =========================================================================
% 5. DYNAMIC WIND SIMULATION
% =========================================================================
% Calculate Aerodynamic Drag Force
drag_force = 0.5 * rho * (wind_speed^2) * Cd * Area;

% Determine Target Angle to Counteract the Wind Force
target_angle_rad = atan(drag_force / drone_weight);
target_angle_deg = target_angle_rad * (180/pi); 

% Simulation Time Setup
dt = 0.01;
time = 0:dt:4;              
n_steps = length(time);

% Initialize state arrays
pitch_angle = zeros(1, n_steps);     
pitch_velocity = zeros(1, n_steps);  

% Simulation Loop
for t = 1:(n_steps - 1)
    current_angle = pitch_angle(t);
    current_vel = pitch_velocity(t);
    
    error = target_angle_deg - current_angle;
    derivative = 0 - current_vel;
    
    control_torque = (Kp * error) + (Kd * derivative);
    
    I_yy_deg = I_yy * (180/pi); 
    angular_accel = control_torque / I_yy_deg;
    
    pitch_velocity(t+1) = pitch_velocity(t) + (angular_accel * dt);
    pitch_angle(t+1) = pitch_angle(t) + (pitch_velocity(t+1) * dt);
end

% Plotting Dynamic Tracking
figure(1);
plot(time, pitch_angle, 'r-', 'LineWidth', 2);
hold on;
plot(time, yline(target_angle_deg), 'k--', 'LineWidth', 1.5); 
grid on;
xlabel('Time (seconds)');
ylabel('Pitch Angle (degrees)');
title(['Drone Response Auto-Stabilizing Against a ' num2str(wind_speed) ' m/s Wind']);
legend('Drone Pitch', 'Required Lean Angle for Position Hold');
hold off;


%% =========================================================================
% 6. MODAL ANALYSIS & RESONANCE
% =========================================================================
% Calculate Area Moment of Inertia
I_area = (pi / 4) * (outer_radius^4 - inner_radius^4);

% Calculate Arm Stiffness (k)
k_arm = (3 * E_modulus * I_area) / (arm_len^3);

% Calculate Equivalent Mass
m_eq = motor_mass + (0.236 * arm_mass);

% Calculate Natural Frequencies
omega_n = sqrt(k_arm / m_eq); 
f_n = omega_n / (2 * pi);     
resonant_RPM = f_n * 60;      

fprintf('\n================ MODAL ANALYSIS ================\n');
fprintf('Arm Stiffness (k): %.2f N/m\n', k_arm);
fprintf('Natural Frequency: %.2f Hz\n', f_n);
fprintf('DANGER ZONE:       %.0f RPM\n', resonant_RPM);
fprintf('-------------------------------------------------\n');

if resonant_RPM > min_flight_rpm && resonant_RPM < max_flight_rpm
    fprintf('CRITICAL WARNING: Resonance occurs within flight RPM range!\n');
    fprintf('Fix: Use thicker carbon fiber (increase I_area) or shorten the arms.\n');
else
    fprintf('STATUS: DESIGN PASSED!\n');
    fprintf('Your frame is stiff enough that resonance occurs outside normal flight speeds.\n');
end
fprintf('=================================================\n');

% Frequency Response Plot
test_rpms = 0:100:40000; 
forcing_freqs = test_rpms * (2 * pi / 60); 
zeta = 0.05; 
freq_ratio = forcing_freqs / omega_n;
amplitude_ratio = 1 ./ sqrt((1 - freq_ratio.^2).^2 + (2 * zeta * freq_ratio).^2);

figure(2);
plot(test_rpms, amplitude_ratio, 'b-', 'LineWidth', 2);
hold on;

fill([min_flight_rpm max_flight_rpm max_flight_rpm min_flight_rpm], ...
     [0 0 max(amplitude_ratio) max(amplitude_ratio)], 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

xline(min_flight_rpm, 'g--', 'Min Hover RPM', 'LabelVerticalAlignment', 'bottom');
xline(max_flight_rpm, 'g--', 'Max Thrust RPM', 'LabelVerticalAlignment', 'bottom');

plot(resonant_RPM, max(amplitude_ratio), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
xline(resonant_RPM, 'r-', 'NATURAL FREQUENCY (DANGER)', 'LabelOrientation', 'horizontal', 'Color', 'r');

grid on;
xlabel('Motor Speed (RPM)');
ylabel('Vibration Amplification (Multiplier)');
title('Drone Arm Frequency Response & Resonance Curve');
legend('Amplitude Response', 'Safe Flight Envelope', 'Resonance Peak');
hold off;