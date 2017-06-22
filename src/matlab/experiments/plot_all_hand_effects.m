% Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
%                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
% Author: Giovanni Saponaro, Atabak Dehban
% CopyPolicy: Released under the terms of the GNU GPL v3.0

% plot tool data
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

%act_keyset = keys(act_map);
%tool_keyset = keys(tool_map);

figure;
c = 0;
fontsize = 8;
% iterate over all actions
for a = 1:num_actions
    % iterate over all manipulators
    for m = 1:num_hands
        c = c + 1;
        subplot(num_actions, num_hands, c);
        scatter(ey{a,m}, -ex{a,m});
        title(string(act_map2(a)) + string(', ') + string(hand_map(m)), 'FontSize',fontsize);
        xlim([-0.20 0.20]);
        ylim([-0.20 0.20]);
        set(gca,'FontSize',fontsize);
        %xlabel('EffectX');
        %ylabel('EffectY');
    end
end

% export to eps
%print('-depsc', 'all_hand_effects_2obj.eps');