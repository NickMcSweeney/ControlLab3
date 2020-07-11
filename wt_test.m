%% Test script for the water tank model %%
% computes a steady state u from a refrence y
% and uses a rk4 integration to plot the resulting change in water level

clear all
close all

% Create an instance of the water tank class
wt = water_tank_model();
wt.setNewWaterLevel(99); % the starting water level

% Time step
dt = 0.1;

% Compute a reference
y_ref = 33; % what you want to water level to become

u = wt.computeSteadyStateControl(y_ref); % compute the u value

nb_steps = 1000; % integration period

% These are only used for plotting the results
vec = zeros(1,nb_steps);

u_vec = vec;
y_vec = vec;
ref_y_vec = vec;

for (i = 1:nb_steps)

    % Query the tank - get sensor readings
    y = wt.getWaterLevel();

    y_vec(i) = y;
    ref_y_vec(i) = y_ref;

    % Update the model
    wt.integrateControlRK4(u, dt);

    u_vec(i) = u;
end

plot(1:nb_steps,u_vec, 'r-', 'DisplayName', 'u- value');
hold on;
plot(1:nb_steps,y_vec, 'b-', 'DisplayName', 'y- measurement');
plot(1:nb_steps,ref_y_vec, 'k-', 'DisplayName', 'y- refrence');
legend
hold off;
