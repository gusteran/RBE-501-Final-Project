classdef Drone
    %DRONE Summary of this class goes here
    %   Detailed explanation goes here
    % Testing
    %Adding comment for test
    properties
        TrueMap
        DroneMap
        X
        Y
        DeltaX double
        DeltaY double
        MaxSpeed
        Acceleration
        KDistance = 0.25;
        DDerivative = 1;
        SensingDistance = 5;
        MaxMoves = 1000;
        Moves = 0;
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

        function boundary = getBoundary(obj)
            obj = obj.sense();
            robotX = round(obj.X)+1;
            robotY = round(obj.Y)+1;
            map = obj.DroneMap == 0;

            for i = 1:obj.SensingDistance

                pixel = robotY+1;
                for i = robotY:size(obj.TrueMap, 1)
                    if map(i, robotX) > 0
                        pixel = i;
                        break;
                    end
                end
                boundary = bwtraceboundary(map, [pixel, robotX], 'N');
    
                for j = 1:size(boundary, 1)
                    map(boundary(j,1), boundary(j,2)) = 0;
                end
            end
%             figure
%             show(occupancyMap(map))
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
            near = ~obj.canMove() | hypot(targetX - obj.X, targetY - obj.Y) < obj.SensingDistance;
        end

        function plot(obj)
            show(occupancyMap(obj.DroneMap));
            hold on
            plot(obj.X, size(obj.TrueMap,1)-obj.Y, "Marker", ".", "MarkerSize",10 * obj.SensingDistance, "Color","red");
            hold off
        end
        
        function hasMoves = canMove(obj)
            hasMoves = obj.Moves < obj.MaxMoves;
        end
    end

    methods (Access = private)
        function obj = move(obj)
%             disp([obj.DeltaX, obj.DeltaY])
            obj.Moves = obj.Moves + 1;
            if obj.Moves < obj.MaxMoves
                obj.X = obj.X + obj.DeltaX;
                obj.Y = obj.Y + obj.DeltaY;
%             else
%                 disp("Out of moves for the experiment!")
            end
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
            if obj.X + deltaX == targetX || obj.Y + deltaY == targetY
                deltaX = deltaX / normDelta * targetSpeed;
                deltaY = deltaY / normDelta * targetSpeed;
            end
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

