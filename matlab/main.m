clc;
clear all;
close all;

%% Define Observation and Action Spaces
obsInfo = rlNumericSpec([4 1], 'LowerLimit', [-Inf; -Inf; -Inf; -Inf], ...
    'UpperLimit', [Inf; Inf; Inf; Inf]);

actInfo = rlNumericSpec([2 1], 'LowerLimit', [-10; -10], ...
    'UpperLimit', [10; 10]);

% Create the environment
env = rlFunctionEnv(obsInfo, actInfo, @cartPendulumStep, @cartPendulumReset);

%% Define Critic Network
criticNet = [
    featureInputLayer(4) 
    fullyConnectedLayer(32) 
    reluLayer
    fullyConnectedLayer(1)]; % Single output for Q-value

critic = rlQValueRepresentation(criticNet, obsInfo, actInfo, ...
    'Observation', {'input'}, 'Action', {'input'});

%% Define Actor Network
actorNet = [
    featureInputLayer(4) 
    fullyConnectedLayer(32) 
    reluLayer
    fullyConnectedLayer(2) % Two outputs for M and F
    tanhLayer
    scalingLayer('Scale', [10; 10])]; % Scale actions within limits

actor = rlDeterministicActorRepresentation(actorNet, obsInfo, actInfo, ...
    'Observation', {'input'});

%% Define and Train the Agent
agentOptions = rlDDPGAgentOptions('SampleTime', 0.01, 'DiscountFactor', 0.99);
agent = rlDDPGAgent(actor, critic, agentOptions);

% Training options
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', 500, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', -0.1);

train(agent, env, trainOpts);