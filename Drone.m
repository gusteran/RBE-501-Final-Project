classdef Drone
    %DRONE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TrueMap
        DroneMap
        X
        Y
        DeltaX
        DeltaY
        MaxSpeed
        Acceleration
        KDistance = 0.25;
        DDerivative = 1;
        SensingDistance = 3;
    end
    
    methods
        function obj = Drone(map, x, y, maxSpeed)
            %DRONE Construct an instance of this class
            %   Detailed explanation goes here
            obj.TrueMap = map;
            obj.DroneMap = zeros(size(map,1), size(map,2));
            obj.X = x;
            obj.Y = y;
            obj.DeltaX = 0;
            obj.DeltaY = 0;
            obj.MaxSpeed = maxSpeed;
            obj.Acceleration = maxSpeed / 5;
            if maxSpeed > 2 * obj.SensingDistance
                disp("Warning!!! Max speed of drone is too great for the sensing capability currently set. The drone may fly faster than its sensing area.")
            end
        end

        function obj = navigateToPoint(obj, targetX, targetY)
            obj = obj.accelerateToPoint(targetX, targetY);
            obj = obj.move();
            obj = obj.sense();
        end

        function obj = navigateWithSpeed(obj, targetX, targetY, speedCoeff)
            obj = obj.accelerate(targetX, targetY, obj.MaxSpeed * speedCoeff);
            obj = obj.move();
            obj = obj.sense();
        end

        function obj = sense(obj)
            if (round(obj.Y) > obj.SensingDistance); y(1) = round(obj.Y)-obj.SensingDistance; else; y(1) = 1; end
            y(2) = round(obj.Y)+obj.SensingDistance;
            if (round(obj.X) > obj.SensingDistance); x(1) = round(obj.X)-obj.SensingDistance; else; x(1) = 1; end
            x(2) = round(obj.X)+obj.SensingDistance;
%             disp("Observe x1 x2; y1 y2")
%             disp([x;y]);
            obj.DroneMap(y(1):y(2), x(1):x(2)) = obj.TrueMap(y(1):y(2), x(1):x(2));
        end

        function near = isNear(obj, targetX, targetY)
            near = hypot(targetX - obj.X, targetY - obj.Y) < obj.SensingDistance;
        end

        function plot(obj)
            show(occupancyMap(obj.DroneMap));
            hold on
            plot(obj.X, size(obj.TrueMap,1)-obj.Y, "*");
            hold off
        end
    end

    methods (Access = private)
        function obj = move(obj)
            obj.X = obj.X + obj.DeltaX;
            obj.Y = obj.Y + obj.DeltaY;
        end

        function obj = accelerateToPoint(obj, targetX, targetY)
            deltaX = (targetX - obj.X) * obj.KDistance;
            deltaY = (targetY - obj.Y) * obj.KDistance;
            accel = [deltaX-obj.DeltaX, deltaY-obj.DeltaY];
            if hypot(accel(1), accel(2)) > obj.Acceleration
                accel = accel / norm(accel) * obj.Acceleration;
            end
            velocity = [obj.DeltaX + accel(1), obj.DeltaY + accel(2)];
            if hypot(velocity(1), velocity(2)) > obj.MaxSpeed
                velocity = velocity / abs(norm(velocity)) * obj.MaxSpeed;
            end
            obj.DeltaX = velocity(1);
            obj.DeltaY = velocity(2);
        end

        function obj = accelerate(obj, targetX, targetY, targetSpeed)
            deltaX = (targetX - obj.X);
            deltaY = (targetY - obj.Y);
            normDelta = norm([deltaX, deltaY]);
            deltaX = deltaX / normDelta * targetSpeed;
            deltaY = deltaY / normDelta * targetSpeed;
            accel = [deltaX-obj.DeltaX, deltaY-obj.DeltaY];
            if hypot(accel(1), accel(2)) > obj.Acceleration
                accel = accel / norm(accel) * obj.Acceleration;
            end
            velocity = [obj.DeltaX + accel(1), obj.DeltaY + accel(2)];
            if hypot(velocity(1), velocity(2)) > obj.MaxSpeed
                velocity = velocity / abs(norm(velocity)) * obj.MaxSpeed;
            end
            obj.DeltaX = velocity(1);
            obj.DeltaY = velocity(2);
        end
    end
end

