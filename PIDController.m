classdef PIDController

    properties
        dt;
        intE;
        lastE;
    end

    methods
        function obj = PIDController(dt)
            obj.dt = dt;
            obj.intE = 0;
            obj.lastE = 0;
        end
        function u = clamp(obj,u)
            if u > 100
                u = 100;
            end
            if u < 0
                u = 0;
            end
        end
        function u = my_pid(obj, e, G)
            de = (e - obj.lastE)/obj.dt;
            obj.lastE = e;

            obj.intE = cat(2,[e], obj.intE);
            %disp(obj.intE);
            int_e = 0;
            if(length(obj.intE) < 20)
                int_e = sum(obj.intE)*obj.dt;
            else
                int_e = sum(obj.intE(1:20))*obj.dt;
            end
            u = G(1) * e + G(2) * int_e + G(3) * de;
            u = clamp(obj,u);
        end
    end
end
