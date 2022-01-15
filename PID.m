function u = PID(error)

persistent previousError;
if isempty(previousError)
    previousError = 0;
end

persistent previousIntegral;
if isempty(previousIntegral)
    previousIntegral = 0;
end

dt = 1;
Kp = 1.2;
Ti = 500;
Kd = 0;

P = Kp * error;

integral = previousIntegral + (error + previousError);
previousIntegral = integral;
I = (1/Ti) * integral * (dt/2);

derivative = (error - previousError) / dt;
previousError = error;
D = Kd * derivative;

u = P + I + D;
end

