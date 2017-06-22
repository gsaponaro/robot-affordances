% Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
%                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
% Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
%         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
%         Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
%         Lorenzo Jamone, Afonso Gonçalves
% CopyPolicy: Released under the terms of the GNU GPL v3.0.

% manipulator selection/ranking with hand->hand network

function quality = select_manipulator_evaluation_hh(action)

fileDesc = '../data+combinator/cont_desc_noheader.txt'; % manipulators(hands) and objects descriptors
fid_desc = fopen(fileDesc,'r');
desc = textscan(fid_desc,'%d %f %f %f %f %f %f %f %f %f %f %f %f'); % manipulatorID descriptors

num_hands = 3;
quality = zeros(1, num_hands);

for h = 1:3

    qualityNumerator = 0;
    qualityDenominator = 0;

    for o = 4:7
        
        indHand = find(desc{1,1}==h); % indexes of rows with hand h
        indObj  = find(desc{1,1}==o);
        
        for Hand = 1:size(indHand,1)
            for Object = 1:size(indObj,1)
                query = zeros(1,25);
                for d = 1:12 % hand descriptor
                    query(1,d) = desc{1,1+d}(indHand(Hand));
                end
                for d = 1:12
                    query(1,12+d) = desc{1,1+d}(indObj(Object));
                end
                query(1,25) = action;

                % query hh network
                posterior = train_hand_select_manipulator(query, 'pca_2n2c2v_hh');
                
                qualityDenominator = qualityDenominator + 1;
                
                if (posterior_evaluation_criterion(posterior,action))
                    qualityNumerator = qualityNumerator + 1;
                end
            end
        end
    end
    
    % put the results in
    % quality(1) straight
    % quality(2) fortyfive
    % quality(3) bent
    quality(h) = qualityNumerator / qualityDenominator;
    
end