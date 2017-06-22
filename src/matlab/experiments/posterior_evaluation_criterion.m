% Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
%                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
% Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
%         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
% CopyPolicy: Released under the terms of the GNU GPL v3.0.

% Evaluates a tool effect posterior probability, returns a boolean if it
% satisfies the criterion for the given action.
%
% Possible criteria:
%
% 'two_vs_two' (default, generous):
%
% tapFromRight => sum of first two columns > sum of last two columns
% tapFromLeft  => sum of last two columns > sum of first two columns
% draw      => sum of last two rows > sum of first two rows
% push     => sum of first two rows > sum of last two rows
%
% 'two_vs_three' (strict):
%
% tapFromRight => sum of first two columns > sum of last three columns
% tapFromLeft  => sum of last two columns > sum of first three columns
% draw      => sum of last two rows > sum of first three rows
% push     => sum of first two rows > sum of last three rows

function output = posterior_evaluation_criterion(posterior,action,criterion)

% if criterion is not specified, use the default criterion
if nargin < 3
    criterion = 'two_vs_two';
end

assert(strcmp(criterion,'two_vs_two') || strcmp(criterion,'two_vs_three'), 'unsupported criterion')

    switch action

        case 1 % tapFromRight
            
            if strcmp(criterion,'two_vs_two')
                output = sum(sum(posterior(:,1:2))) > sum(sum(posterior(:,end-1:end)));
            elseif strcmp(criterion,'two_vs_three')
                output = sum(sum(posterior(:,1:2))) > sum(sum(posterior(:,end-2:end)));
            end
            
        case 2 % tapFromLeft

            if strcmp(criterion,'two_vs_two')
                output = sum(sum(posterior(:,end-1:end))) > sum(sum(posterior(:,1:2)));
            elseif strcmp(criterion,'two_vs_three')
                output = sum(sum(posterior(:,end-1:end))) > sum(sum(posterior(:,1:3)));
            end
            
        case 3 % draw

            if strcmp(criterion,'two_vs_two')
                output = sum(sum(posterior(end-1:end,:))) > sum(sum(posterior(1:2,:)));
            elseif strcmp(criterion,'two_vs_three')
                output = sum(sum(posterior(end-1:end,:))) > sum(sum(posterior(1:3,:)));
            end
            
        case 4 % push

            if strcmp(criterion,'two_vs_two')
                output = sum(sum(posterior(1:2,:))) > sum(sum(posterior(end-1:end,:)));
            elseif strcmp(criterion,'two_vs_three')
                output = sum(sum(posterior(1:2,:))) > sum(sum(posterior(end-2:end,:)));
            end
            
        otherwise

            error('unknown action!');
            output = -1;
            
    end
        
end