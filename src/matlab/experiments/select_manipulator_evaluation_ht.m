% Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
%                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
% Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
%         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
%         Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
%         Lorenzo Jamone, Afonso Gonçalves
% CopyPolicy: Released under the terms of the GNU GPL v3.0.

% manipulator selection/ranking with hand->tool network

function quality = select_manipulator_evaluation_ht(action)

fileDesc = '../data+combinator/tool_cont_desc_noheader.txt'; % manipulators(tools) and objects descriptors
fid_desc = fopen(fileDesc,'r');
desc = textscan(fid_desc,'%d %f %f %f %f %f %f %f %f %f %f %f %f'); % manipulatorID descriptors

num_tools = 3;
quality = zeros(1, num_tools);

for t = 8:10

    qualityNumerator = 0;
    qualityDenominator = 0;

    for o = 4:7
        
        indTool = find(desc{1,1}==t); % indexes of rows with tool t
        indObj  = find(desc{1,1}==o);
        
        for Tool = 1:size(indTool,1)
            for Object = 1:size(indObj,1)
                query = zeros(1,25);
                for d = 1:12 % tool descriptor
                    query(1,d) = desc{1,1+d}(indTool(Tool));
                end
                for d = 1:12
                    query(1,12+d) = desc{1,1+d}(indObj(Object));
                end
                query(1,25) = action;

                % query ht network
                posterior = train_hand_select_manipulator(query, 'pca_2n2c2v_ht');
                
                qualityDenominator = qualityDenominator + 1;
                
                % generous two_vs_two criterion
                %if (posterior_evaluation_criterion(posterior,action))
                % strict two_vs_three criterion
                if (posterior_evaluation_criterion(posterior,action,'two_vs_three'))
                    qualityNumerator = qualityNumerator + 1;
                end
            end
        end
    end
    
    % put the results in
    % quality(1) hook
    % quality(2) stick
    % quality(3) rake
    quality(t-7) = qualityNumerator / qualityDenominator;
    
end