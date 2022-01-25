clc;
close all;
clear all;

%import data
data = importdata('results.txt');
temp = zeros(1,length(data)/3);
setPoint = zeros(1,length(data)/3);
duty = zeros(1,length(data)/3);
j = 1;
for i = 1:3:length(data)
    temp(j) = data(i);
    setPoint(j) = data(i + 1);
    duty(j) = data(i + 2);
    j = j + 1;
end

%set Tolerance
tolerance = 5;

%calculating u and error
u = duty/999.0;
error = temp - setPoint;
t = 1:1:length(temp);


overshoot = max(temp) - setPoint(1);
relativeOvershoot = (overshoot/setPoint(1))*100;
potentialRegulation = 0;
boolean = 0;
for i = 1:1:length(temp)
    if ((((abs(temp(i) - setPoint(i))/setPoint(i)) * 100)) < tolerance) & boolean == 0
        boolean = 1;
        potentialRegulation = t(i);
    elseif boolean == 1 & ((((abs(temp(i) - setPoint(i))/setPoint(i)) * 100)) > tolerance)
        boolean = 0;
    end
end
regulationCost = sum(u.*u);

%plots
figure;
plot(t,temp);
xlabel('t');
ylabel('temperature');
hold on;
plot(t,setPoint);
xlabel('t');
ylabel('setPoint');
figure;
plot(t,u)
xlabel('t');
ylabel('u');
figure;
plot(t,error)
xlabel('t');
ylabel('error');