classdef water_tank_model < handle
    % WATER_TANK_MODEL Simple single water tank module
    %   Simulates a single water tank module

    properties (Access = private)
        A % Cylinder area
        a % Drain hole area
        gamma % Control input to flow parameter

        y % current measurement
        y_old % previous measurement
        dt % time diff between previous and current

        u % current control value
    end

    methods
        function obj = water_tank_model()
            %WATER_TANK_MODEL Construct an instance of this class
            %   Default parameters are used
            obj.A = 0.008;
            obj.a = 0.0021; % 0.001
            obj.gamma = 0.01;

            obj.y = 0;
            obj.y_old = 0;
            obj.dt = 0.1;
            obj.u = 0;
        end

        function setNewWaterLevel(obj, y)
            %SETNEWWATERLEVEL Set a new waterlevel, this is only supposed
            %to be used to set the system to different state and not to be
            %used withing any control loops.
            obj.y = y;
            obj.y_old = y;
        end

        function setParams(obj, A, a, gamma)
           obj.A = A;
           obj.a = a;
           obj.gamma = gamma;
        end


        function dy = changeInWaterLevel(obj, y, u)
            %CHANGEINWATERLEVEL The derivative of y.
            % The derivative is based on the flow through the drainage (the height of water -> water
            % pressure -> a*sqrt(2*g*y)) and amount of pumped water (u*gamma)

            % Check - can't have negative height
            if (y < 0)
                dy = 0;
                return;
            end

            % Check - if the height > 100 then the water will go through
            % the "overflow pipes".
            if (y > 100)
                dy = 0;
                return;
            end

            dy = (-obj.a*sqrt(2*9.82*y) + u*obj.gamma)/obj.A;
        end

        function y = getWaterLevel(obj)
            y = obj.y;
        end

        function dy = getWaterLevelChange(obj)
           dy = (obj.y - obj.y_old)/obj.dt
        end

        function y = tankLevelLimits(obj, y)
            if (y > 100)
                y = 100;
            end
            if (y < 0)
                y = 0;
            end
        end

        function u = controlLimits(obj, u)
            if (u < 0)
                u = 0;
            end
            if (u > 100)
                u = 100;
            end
        end

        function y_new = integrateControlRK4(obj, u, dt)
            %INTEGRATECONTROL Integrate control action u for time dt using
            %RK4 (Runge-Kutta order 4).
            u = controlLimits(obj, u);
            y_new = 0;
            k1 = dt*changeInWaterLevel(obj, obj.y, u);
            k2 = dt*changeInWaterLevel(obj, obj.y+k1/2, u);
            k3 = dt*changeInWaterLevel(obj, obj.y+k2/2, u);
            k4 = dt*changeInWaterLevel(obj, obj.y+k3, u);

            y_new = obj.y + (k1+2*k2+2*k3+k4)/6;
            y_new = tankLevelLimits(obj, y_new);
            obj.y_old = obj.y;
            obj.dt = dt;
            obj.y = y_new;
        end

        function y_new = integrateControlEuler(obj, u, dt)
            %INTEGRATECONTROLEULER Integrate contol action u for time dt
            %using euler's integration.
            u = controlLimits(obj,u);
            y_new = obj.y + dt*changeInWaterLevel(obj, obj.y, u);

            y_new = tankLevelLimits(obj, y_new);
            obj.y_old = obj.y;
            obj.dt = dt;
            obj.y = y_new;
        end

        function u_ss = computeSteadyStateControl(obj, y)
           %COMPUTESTEADYSTATECONTROL Given a water tank level, compute a steady state control value that according to the model will bring the water level to y.
           dy_ss = 0;
           u_ss = (dy_ss + obj.a * sqrt(2*9.81*y)) / obj.gamma;
        end

        function [ref_y, ref_u] = computeReferenceSteadyStateControl(obj, y, nb_steps)
            u_ss = obj.computeSteadyStateControl(y);
            ref_y = y*ones(1,nb_steps);
            ref_u = u_ss*ones(1,nb_steps);
        end

        function [ref_y, ref_u] = computeSinReferenceAndControl(obj, nb_steps, dt)
            % Here we add some control values together using the water tank model to get the reference value.
            % Typically you would like to compute the required control
            % values to reach a specific state.
            ref_y = zeros(1,nb_steps);
            ref_u = zeros(1,nb_steps);

            setNewWaterLevel(obj,10);

            for (i = 1:nb_steps)
                u = 30 + 20*sin(i/10);

                ref_y(i) = getWaterLevel(obj);
                integrateControlRK4(obj, u, dt);
                ref_u(i) = u;

            end

            setNewWaterLevel(obj,35);
        end

        function [ref_y, ref_u] = computeTrajectoryReferenceAndControl(obj, y, nb_steps, dt, wl)
            % Here we add some control values together using the water tank model to get the reference value.
            % Typically you would like to compute the required control
            % values to reach a specific state.
            ref_y = zeros(1,nb_steps);
            ref_u = zeros(1,nb_steps);

            setNewWaterLevel(obj,wl);

            for (i = 1:nb_steps)
                x = i / 20;
                cur_y = wl+(((1+exp(-(1.42*x))))^-4)*(y-wl);
                u = computeSteadyStateControl(obj,cur_y);
                ref_y(i) = cur_y;
                integrateControlRK4(obj, u, dt);
                ref_u(i) = u;

            end
            setNewWaterLevel(obj,wl);
        end

        function [ref_y, ref_u] = computeOnOffReferenceAndControl(obj, y, nb_steps, dt)
            % Here we add some control values together using the water tank model to get the reference value.
            % Typically you would like to compute the required control
            % values to reach a specific state.
            ref_y = zeros(1,nb_steps);
            ref_u = zeros(1,nb_steps);

            setNewWaterLevel(obj,15);

            for (i = 1:nb_steps)
                u = 0;%computeSteadyStateControl(obj,y);
                if getWaterLevel(obj) < y-1
                    u = 100;
                end
                ref_y(i) = getWaterLevel(obj);
                integrateControlRK4(obj, u, dt);
                ref_u(i) = u;

            end
            setNewWaterLevel(obj,5);
        end
        function [ref_y, ref_u] = computePIDReferenceAndControl(obj, y, nb_steps, c, dt)
            % Here we add some control values together using the water tank model to get the reference value.
            % Typically you would like to compute the required control
            % values to reach a specific state.
            ref_y = zeros(1,nb_steps);
            ref_u = zeros(1,nb_steps);

            setNewWaterLevel(obj,15);

            %G = [9 3 6];
            G = [5 0.1 3];
            for (i = 1:nb_steps)
                e = y - getWaterLevel(obj);
                u = c.my_pid(e,G);
                ref_y(i) = getWaterLevel(obj);
                integrateControlRK4(obj, u, dt);
                ref_u(i) = u;

            end
            setNewWaterLevel(obj,5);
        end
    end

end
