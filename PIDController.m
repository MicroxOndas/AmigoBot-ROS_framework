classdef PIDController < handle
%PIDCONTROLLER Generic discrete-time PID controller with anti-windup.
%
%  CONSTRUCTOR
%    pid = PIDController(Kp, Ki, Kd)
%    pid = PIDController(Kp, Ki, Kd, 'MaxOutput', 1.5, ...
%                                    'MinOutput', -1.5, ...
%                                    'MaxIntegral', 5.0)
%
%  METHODS
%    output = pid.compute(error, dt)  - Compute saturated PID output
%    pid.reset()                      - Reset integral and derivative state
%    pid.setGains(Kp, Ki, Kd)         - Update gains (resets internal state)

    properties (Access = public)
        Kp           % Proportional gain
        Ki           % Integral gain
        Kd           % Derivative gain
        MaxOutput    % Output upper saturation limit
        MinOutput    % Output lower saturation limit
        MaxIntegral  % Anti-windup: maximum magnitude of integral term
    end

    properties (Access = private)
        integral_     % Accumulated integral value
        prev_error_   % Previous error for backward-difference derivative
        initialized_  % True after the first compute() call
    end

    methods
        % -----------------------------------------------------------------
        function obj = PIDController(Kp, Ki, Kd, varargin)
            p = inputParser();
            addRequired(p, 'Kp', @isnumeric);
            addRequired(p, 'Ki', @isnumeric);
            addRequired(p, 'Kd', @isnumeric);
            addParameter(p, 'MaxOutput',    Inf, @isnumeric);
            addParameter(p, 'MinOutput',   -Inf, @isnumeric);
            addParameter(p, 'MaxIntegral',  Inf, @isnumeric);
            parse(p, Kp, Ki, Kd, varargin{:});

            obj.Kp          = p.Results.Kp;
            obj.Ki          = p.Results.Ki;
            obj.Kd          = p.Results.Kd;
            obj.MaxOutput   = p.Results.MaxOutput;
            obj.MinOutput   = p.Results.MinOutput;
            obj.MaxIntegral = p.Results.MaxIntegral;
            obj.reset();
        end

        % -----------------------------------------------------------------
        function output = compute(obj, error, dt)
        %COMPUTE Calculate the PID control output for the current timestep.
        %
        %  error  - Signed error signal (setpoint - measurement)
        %  dt     - Elapsed time since last call [s]
        %  output - Saturated control output

            dt = max(dt, 1e-6);  % Guard against zero division

            % Seed derivative on first call to avoid a spike
            if ~obj.initialized_
                obj.prev_error_ = error;
                obj.initialized_ = true;
            end

            % --- Integral with symmetric anti-windup clamping ---
            obj.integral_ = obj.integral_ + error * dt;
            obj.integral_ = max(-obj.MaxIntegral, ...
                            min( obj.MaxIntegral, obj.integral_));

            % --- Backward-difference derivative ---
            derivative = (error - obj.prev_error_) / dt;

            % --- PID sum and output saturation ---
            raw    = obj.Kp * error ...
                   + obj.Ki * obj.integral_ ...
                   + obj.Kd * derivative;
            output = max(obj.MinOutput, min(obj.MaxOutput, raw));

            obj.prev_error_ = error;
        end

        % -----------------------------------------------------------------
        function reset(obj)
        %RESET Clear the integral accumulator and derivative memory.
            obj.integral_    = 0;
            obj.prev_error_  = 0;
            obj.initialized_ = false;
        end

        % -----------------------------------------------------------------
        function setGains(obj, Kp, Ki, Kd)
        %SETGAINS Update PID gains and reset the controller state.
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.reset();
        end
    end
end
