clc;
close all;
clear all;

%prepare data
openLoopAnswer = importdata('openLoopAnswer.txt');
openLoopAnswerData = openLoopAnswer.data;
temperature = openLoopAnswerData(:);
dt = 1;
t = (0:1:length(openLoopAnswerData) - 1) * dt;
ref = 37;
duty = 1;
input = duty * ones(1,length(openLoopAnswerData));

%model
s = tf('s');
c = temperature(1);
k = 21.39786113868873/duty;
T = 248.9986821828368;
G = k/(1+s*T);

%model open loop response
modelResponseOpenLoop = lsim(G,input,t);
modelResponseOpenLoop = modelResponseOpenLoop + c;

%model error
residuum = temperature - modelResponseOpenLoop;
errorSum = sum(abs(residuum))

%plots
figure;
plot(t,temperature);
hold on;
plot(t,modelResponseOpenLoop)

figure;
plot(t,residuum);


model = sim("model.slx");
tempRef = model.get('tempRef');
tempReal = model.get('tempReal');
t = model.get('tout');

figure;
%plot(t,tempRef);
hold on;
plot(t,tempReal);