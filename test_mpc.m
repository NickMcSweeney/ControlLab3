clear all
close all

% Create an instance of the water tank class
wt = water_tank_model();

% Create an instance of the controller
controller = model_predictive_control();
wt.setNewWaterLevel(10);

% Time step
dt = 0.1;

c = PIDController(dt);
% Compute a reference
%[y_ref, u_ref] = wt.computeReferenceSteadyStateControl(50, 200);
%[y_ref, u_ref] = wt.computeSinReferenceAndControl(200, dt);
[y_ref, u_ref] = wt.computeTrajectoryReferenceAndControl(50,200,dt,10);
%[y_ref, u_ref] = wt.computeOnOffReferenceAndControl(50,200,dt);
%[y_ref, u_ref] = wt.computePIDReferenceAndControl(50,200,c,dt);
u = 0;

% Length of the preview window
D = 20;

% The weight matrices
controller.setStateWeight(4);
controller.setControlWeight(0.1);

nb_steps = length(y_ref)-D;

% These are only used for plotting the results
u_vec = zeros(1,nb_steps);
y_vec = zeros(1,nb_steps);
ref_y_values = zeros(1,nb_steps);
ref_u_values = zeros(1,nb_steps);

for (i = 1:nb_steps)

    % Query the tank - get sensor readings
    y = wt.getWaterLevel();

    horizon_y_ref = y_ref(i:i+D-1);
    horizon_u_ref = u_ref(i:i+D-1);

    y_vec(i) = y;
    ref_y_values(i) = horizon_y_ref(1);
    ref_u_values(i) = horizon_u_ref(1);
    u = controller.computeControl(horizon_y_ref, horizon_u_ref, y, dt);

    % Update the model
    wt.integrateControlRK4(u, dt);

    u_vec(i) = u;
end

figure
plot(1:nb_steps,ref_u_values, 'g-');
hold on
plot(1:nb_steps,u_vec, 'r-');
plot(1:nb_steps,y_vec, 'b-');
plot(1:nb_steps,ref_y_values, 'k-');

legend('u- reference', 'u- control', 'y- measurement', 'y- reference')
hold off;
