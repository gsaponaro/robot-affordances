% Copyright: (C) 2019 VisLab, Institute for Systems and Robotics,
%                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
% Author: Giovanni Saponaro, Atabak Dehban
% CopyPolicy: Released under the terms of the GNU GPL v3.0

% 


file_hand_eff = '../data+combinator/cont_eff_noheader.txt';
fid_hand_eff  = fopen(file_hand_eff, 'r');
eff  = textscan(fid_hand_eff, '%d %d %s %f %f %f %f %f %f %f %f'); % hand obj action initpos3d initpos2d finpos3d finpos2d

% convert action string from cell to char
eff{1,3} = char(eff{1,3});

act_map = containers.Map;
act_map('tapFromRight') = 1;
act_map('tapFromLeft') = 2;
act_map('draw') = 3;
act_map('push') = 4;

% num_actions = act_map.size(1);
act_map2 = {'tapFromRight' 'tapFromLeft' 'draw' 'push'};
num_actions = size(act_map2,2);

% tool_map = containers.Map;
% tool_map('hook') = 8;
% tool_map('stick') = 9;
% tool_map('rake') = 10;
% num_tools = tool_map.size(1);
%hand_map = {'straight', 'fortyfive', 'bent'};
hand_map = {'straight', 'arched', 'bent'};
num_hands = size(hand_map,2);

% initialize two cell arrays with all elements set to [], where
% we will store EffectX and EffectY for plotting
[ex{1:num_actions, 1:num_hands}] = deal([]);
[ey{1:num_actions, 1:num_hands}] = deal([]);

% iterate over experiments
for exp = 1:size(eff{1,1},1)
    
    % current hand ID: 1 2 or 3
    indHand = eff{1,1}(exp);
    indCol = indHand;
    
    % current EffectX to save
    new_ex = eff{1,8}(exp) - eff{1,4}(exp);
    
    % current EffectY to save
    new_ey = eff{1,9}(exp) - eff{1,5}(exp);

    % current action index, obtained by scanning eff{1,3}
    act_type = act_map(strtrim(eff{1,3}(exp,:))); % 1 2 3 4

    % save the effects to the appropriate cells {action}{tool}
    ex{act_type,indCol} = [ex{act_type,indCol} new_ex];
    ey{act_type,indCol} = [ey{act_type,indCol} new_ey];

end
clear exp;

% ranges for effect descritization
ranges2 = cell(2,1);
ranges2{1,1} = [-.06 -0.025 0.025 0.06 1]; %best
ranges2{2,1} = [-.06 -0.025 0.025 0.06 1]; %best

% consider only action draw (pull)
action_index = 3;

% iterate over all manipulators
for m = 1:num_hands
    fprintf("ex{draw,%d}\n", m);
    exps = ex{action_index,m};
    
    % count total number of experiments in this action-manipulator pair
    num_exps = length(exps);
    
    % count how many experiments will fall in desired bin according to
    % human ground truth (goal of the action)
    good = 0;

    % iterate over all experiments for selected action-manipulator pair
    for e = 1:num_exps

        value = exps(e);

        % invert sign because we plot x axis opposite to robot reference
        % (if we don't invert, we would have to compute in robot r.f.)
        value = -value;

        fprintf("\texp. %d: %f -> ", e, value);

        % using default 'two_vs_two' generous criterion (see
        % posterior_evaluation_criterion.m).
        % check if value is within the first two bins
        if ((value < ranges2{1,1}(1)) || (value < ranges2{1,1}(2)))
            fprintf("good");
            good = good+1;
        else
            fprintf("bad");
        end

        fprintf("\n");
    
    end

    fprintf("\ttotal ground truth: %d/%d (%f)\n", good, num_exps, good/num_exps);
    
    % note: y direction does not count for draw action ground truth,
    %       so we do not consider current_ey = ey{action_index,m}
end