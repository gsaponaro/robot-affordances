% Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
%                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
% Author: Afonso Gonçalves, Pedro Vicente, Giovanni Saponaro, Atabak Dehban
% CopyPolicy: Released under the terms of the GNU GPL v3.0

function combinatorWithNoise_handdata(fileDesc, fileEff, fileAff)

fileDesc = 'cont_desc_noheader.txt';
fileEff = 'cont_eff_noheader.txt';
fileAff = 'affData_03_2017.txt';

fid_desc = fopen(fileDesc,'r');
fid_eff  = fopen(fileEff, 'r');
%fid_aff  = fopen(fileAff, 'w');
dlmwrite(fileAff, [], 'delimiter','');

noiseRange = 0.02; % +/-0.02 m

desc = textscan(fid_desc,'%d %f %f %f %f %f %f %f %f %f %f %f %f'); % manipulator(tool/hand)ID descriptors
eff  = textscan(fid_eff, '%d %d %s %f %f %f %f %f %f %f %f'); % hand obj action initpos3d initpos2d finpos3d finpos2d
% desc{1,1}=char(desc{1,1});
% eff{1,1}=char(eff{1,1});
% eff{1,2}=char(eff{1,2});
eff{1,3}=char(eff{1,3});

% hand_obj_map = containers.Map;
% hand_obj_map('straight ')=1;
% hand_obj_map('fortyfive')=2;
% hand_obj_map('bent     ')=3;
% 
% hand_obj_map('pear     ')=1;
% hand_obj_map('lemon    ')=2;
% hand_obj_map('wball    ')=3;
% hand_obj_map('ylego    ')=4;
% 
act_map = containers.Map;
act_map('draw')=3;
act_map('push')=4;
act_map('tapFromLeft')=2;
act_map('tapFromRight')=1;

for n = 1:size(eff{1,1},1)
    
    indHand   = find(desc{1,1}==eff{1,1}(n));
    indObj = find(desc{1,1}==eff{1,2}(n));
    
    temp_aff(1:(size(indHand,1)*size(indObj,1)),1:27) = NaN;
    
    for Tool = 1:size(indHand,1)
        for Target = 1:size(indObj,1)
            for d = 1:12 % tool descriptor
                temp_aff(Target+(Tool-1)*size(indObj,1),d) = desc{1,d+1}(indHand(Tool));
            end
            for d = 1:12
                temp_aff(Target+(Tool-1)*size(indObj,1),d+12) = desc{1,d+1}(indObj(Target));
            end
            temp_aff(Target+(Tool-1)*size(indObj,1),25) = act_map(strtrim(eff{1,3}(n,:)));% action
            temp_aff(Target+(Tool-1)*size(indObj,1),26) = (eff{1,8}(n) - eff{1,4}(n)) + ( rand*noiseRange*2 - noiseRange); %effectX + uniform noise
            temp_aff(Target+(Tool-1)*size(indObj,1),27) = (eff{1,9}(n) - eff{1,5}(n)) + ( rand*noiseRange*2 - noiseRange); %effectY + uniform noise
            
        end     
    end  
    dlmwrite(fileAff, temp_aff, 'delimiter',' ','-append');
    clear temp_aff
end

end