% Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
%                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
% Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
%         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
%         Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
%         Lorenzo Jamone, Afonso Gonçalves
% CopyPolicy: Released under the terms of the GNU GPL v3.0.

% effect prediction queries are in the following form:
% p(EffectX, EffectY | disc(pca(T_or_H)), disc(pca(O)), A),
% where
% EffectX and EffectY are the indexes of the BN observed nodes (e.g., 6 and
% 7); disc(pca(T_or_H)) are the features of the Tool or Hand having been
% processed by PCA and then discretized; disc(pca(O)) is the same for the
% target Object; A is the index of the given motor action.
%
% Output:
%   5x5 matrix of probabilities distribution
%   p(EffectX, EffectY | nodeName_1=nodeValue_1, ... , nodeName_i=nodeValue_i)

function posterior = train_hand_select_manipulator(query, whichbn)

%initPmtk3;

%% Select the network, or choose the default one

% if none specified, set default network
if nargin < 2
    whichbn = 'pca_2n2c2v_ht';
end

% confirm that whichbn is one of the permitted ones
assert(strcmp(whichbn,'pca_2n2c2v_ht') || strcmp(whichbn,'pca_2n2c2v_hh'), 'you must specify a permitted network')

%fprintf('selected Bayesian Network: %s\n', whichbn);

%% Verify input format
sizeT = 12; % actually Tool or Hand
sizeO = 12;
sizeA =  1;
expSize = sizeT + sizeO + sizeA;
assert(length(query)==expSize, 'query must have %d elements (Tool + Object + Action)', expSize);

%% Load network variables
%BN, components, pcO, pcH/pcT, ranges, mu_train, sigma_train
switch whichbn

    case {'pca_2n2c2v_ht', 'pca_2n2c2v_hh'}
        load([ '../data+combinator/' whichbn '.mat' ]);
        %fprintf('loaded Bayesian Network: %s\n', whichbn);

    otherwise
        error([whichbn ' is not a known Bayesian Network']);
        
end

%% For figure visualization
% figWidth = 400;  % 450
% figHeight = 380; % 430
% figLeftDefault = 1900; %2000
% figBottomDefault = 0; %50
% 
% figIndexX_MAX = 2;
% figIndexY_MAX = 1;
% 
% figLeft = figLeftDefault;
% figBottom = figBottomDefault;
%
% queryActionID = 3;
% queryActionName = 'draw';

%% Process query
switch whichbn

    case {'pca_2n2c2v_ht', 'pca_2n2c2v_hh'}

        prior_nodes  = [1 2 3 4 5]; % pca1_T pca2_T pca3_O pca4_O action
        features(1:sizeT+sizeO) = query(1:sizeT+sizeO); % features of the tool and object
%         features = (features - mu_train) ./ sigma_train; % normalization w.r.t. training mu,sigma; only works for >MATLAB2016b
        features=bsxfun(@minus, features(:,1:24), mu_train);
        features=bsxfun(@rdivide, features(:,1:24), sigma_train);
        action = query(end);
        featuresT = features(1:sizeT);
        featuresO = features(sizeT+1:sizeT+sizeO);
        % linear projection, which in pmtk3 PCA we must do it this way
        % (with MATLAB PCA it could be done by calling
        % [coeff,*score*,latent] = pca(___))
        score = [featuresT*pinv(pcH(:,1:components)') featuresO*pinv(pcO(:,1:components)')];
        score = discretize(score,ranges);
        prior_values = [score action]; % and add the action to prior
        posterior_nodes = [6 7]; % EffectX and EffectY
     
    otherwise
        error([whichbn ' is not a known Bayesian Network']);
        
end

%% Set the priors and do the query p(posteriors|priors):
clamped = sparsevec(prior_nodes, prior_values, BN.nnodes);
prob = dgmInferQuery(BN, posterior_nodes, 'clamped', clamped, ...
                     'doSlice', false);

%disp('posterior probability distribution');
%prob.T

posterior = prob.T;



%% put result in yarp Bottle format
%answer_string = '';
%answer_string = [answer_string '('];
%for k = 1:size(prob.T,1)
%    answer_string = [answer_string '('];
%    for j=1:size(prob.T,1)
%        answer_string = [answer_string ' ' num2str(prob.T(k,j))];
%    end
%    answer_string = [answer_string ') '];
%end
%answer_string=[answer_string ')'];
%answer_string


%% OLD DOCUMENTATION FROM poeticon prediction.m
%% Effect prediction, queries server
% Inputs (standard yarp port):
%    . Bottle1x11 (or 14) vector with prior - Just doubles
%        5 desc for the tool   (toolEffector)
%        5 desc for the object (whole object descriptors)
%        1 action
%        optional: 3 strings ( object, tool, hand)
%    . displayON, displayOFF - Turn ON/OFF the display figure
%
%    . sameON, sameOFF       - reutilize figure 1 (ON) or create a figure
%                              for each query (OFF)
% Output (standard yarp port):
%   5x5 matrix of probabilities distribution
%   p(EffectX, EffectY | nodeName_1=nodeValue_1, ... , nodeName_i=nodeValue_i)
%
% NOTE: Choose the Network (bn). The default Network is 2n_2c_2v
% n: number of PCA
%        e.g 1 for tool and 1 for object
% c: number of PCA components used in each PCA. 
%        e.g 2components in PCA_tool and 2 in PCA_object
% v: number of values/bins for discretization of each PCA component. 
%       e.g v=2 - PCA_tool1 divided in two values/bins, PCA_tool2 divided
%       in two, ...., etc


%% figure stuff
% x = [0.5,1.5,2.5,3.5,4.5];
%             figIndexX = mod ((i-1),figIndexX_MAX);
%             %figIndexX
%             
%             figIndexY = mod ( int32(fix((i-1)/figIndexX_MAX)) ,figIndexY_MAX);
%             %figIndexY
%             
%             %figLeft = figLeftDefault + (i-1)*figWidth;
%             figLeft = figLeftDefault + figIndexX*figWidth;
%             %figLeft
%             figBottom = figBottomDefault + figIndexY*figHeight;
%             %figBottom
%             
%             if (action == 1.0)
%                 queryActionName = 'draw';
%             elseif (action == 2.0)
%                 queryActionName = 'push';
%             end
%             if(display)
%                 if(sameFig)
%                     hFig(i)=figure(1);
%                 else
%                     hFig(i) = figure(i);
%                 end
%                 str = sprintf('%s action (Query #%d)',queryActionName, i); 
%                 if(query.size()==14)
%                     str = sprintf('%s %s with %s on %s (Query #%d)',queryActionName, char(query.get(11).asString), char(query.get(12).asString),char(query.get(13).asString),i); 
%                 end
%                 title(str);
%                 set(hFig(i), 'Position', [figLeft figBottom figWidth figHeight]) % maybe change the position of the window
%                 axis([0 5 0 5.5])
%                 hold on;
%                 scatter (x, 0.5*ones(1,5) , 5000, prob.T(1,:),'filled','s')
%                 hold on;
%                 scatter (x, 1.5*ones(1,5) , 5000, prob.T(2,:),'filled','s')
%                 hold on;
%                 scatter (x, 2.5*ones(1,5) , 5000, prob.T(3,:),'filled','s')
%                 hold on;
%                 scatter (x, 3.5*ones(1,5) , 5000, prob.T(4,:), 'filled','s')
%                 hold on;
%                 scatter (x, 4.5*ones(1,5) , 5000, prob.T(5,:),'filled','s')
%                 hold on;
%                 colormap gray;
%                 plot(2.5,5.25,'r*','LineWidth',8) ; % Display robot position
%                 pause(1);
%                 i=i+1;
%             end
%             portOutput.write(answer);
%             disp('Done');            
%         end
%     end
%     pause(0.01);
% end
% disp('Going to close the ports');
% portInput.close;
% portOutput.close;
% close all;
% clear;



