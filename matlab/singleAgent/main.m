clc;
clear all;

env = singleAgentCartNPendulum();

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

disp(obsInfo);
disp(actInfo);

% Define the critic (Q-network)
criticNet = [
    featureInputLayer(prod(obsInfo.Dimension), 'Name', 'state')
    fullyConnectedLayer(128, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(128, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(numel(actInfo.Elements), 'Name', 'output')
];



% Define the critic (Q-value function)
critic = rlVectorQValueFunction(criticNet, obsInfo, actInfo);

agentOpts = rlDQNAgentOptions( ...
    'SampleTime', 0.01, ...
    'DiscountFactor', 0.99, ...
    'EpsilonGreedyExploration', rl.option.EpsilonGreedyExploration( ...
        'Epsilon', 1, ...
        'EpsilonDecay', 0.995, ...
        'EpsilonMin', 0.01), ...
    'ExperienceBufferLength', 1e6, ...
    'MiniBatchSize', 64, ...
    'TargetSmoothFactor', 1e-3);



agent = rlDQNAgent(critic, agentOpts);

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 10000, ...
    'MaxStepsPerEpisode', 500, ...
    'Verbose', true, ...
    'Plots', 'training-progress', ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 195);


trainingStats = train(agent, env, trainOpts);

timestamp = datestr(now, 'yyyymmdd_HHMMSS'); % Get current date and time
filename = ['trainedAgent_' timestamp '.mat']; % Create a unique filename
save(filename, 'agent', 'env'); % Save with timestamped filename

plot(env);

simOptions = rlSimulationOptions(MaxSteps=500);
experience = sim(env,agent,simOptions);