classdef model_predictive_control < handle
    %MODEL PREDICTIVE CONTROL (MPC) based control using
    %a prediction window - used in the water tank example


    properties

        A % Cylinder area
        a % Drain hole area
        gamma % Control input to flow parameter

        r % Control weigh scalar
        q % State weight scalar
    end

    methods
        function obj = model_predictive_control
            %STATE_FEEDBACK_CONTROL Construct an instance with default parameters

            obj.A = 0.008;
            obj.a = 0.0021;
            obj.gamma = 0.01;

            obj.r = 0.1;
            obj.q = 1;
        end

        function setParams(obj, A, a, gamma)
           obj.A = A;
           obj.a = a;
           obj.gamma = gamma;
        end


        function setStateWeight(obj, q)
            obj.q = q;
        end

        function setControlWeight(obj, r)
            obj.r = r;
        end

        function u = computeControl(obj, horizon_y_ref, horizon_u_ref, current_y, dt)

            if (length(horizon_y_ref) ~= length(horizon_u_ref))
                error("difference in the horizon length");
            end

            D = length(horizon_y_ref);

            [S T] = compute_ST(horizon_y_ref, horizon_u_ref, dt, obj.A, obj.a, obj.gamma);

            y0 = current_y - horizon_y_ref(1)

            R = eye(D)*obj.r;
            Q = eye(D)*obj.q;

            [P p] = compute_Pp(S,T,R,Q, y0);

            % TODO add the constraints matrix computations later on here
            H = eye(D);
            h = ones(D,1)*30;
            u_vec = quadprog(2.*P,p,H,h);

            % Compute the control actions...
            %u_vec = quadprog(2.*P,p);

            if (length(u_vec) == 0)
                error("couldn't find a feasible solution");
                u = 0;
            else
                idx = 1;
                u_vec
                horizon_u_ref
                u = u_vec(idx)+horizon_u_ref(idx)

            end

        end



    end
end
