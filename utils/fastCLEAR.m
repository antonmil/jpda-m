function combinedMets=fastCLEAR(allMets, nGT, nTR, Fgt)
% combine fast all metrics
% MOTP is approximate!!!

% metrics contains the following
% [1]   recall	- recall = percentage of detected targets
% [2]   precision	- precision = percentage of correctly detected targets
% [3]   FAR		- number of false alarms per frame
% [4]   GT        - number of ground truth trajectories
% [5-7] MT, PT, ML	- number of mostly tracked, partially tracked and mostly lost trajectories
% [8]   falsepositives- number of false positives (FP)
% [9]   missed        - number of missed targets (FN)
% [10]  idswitches	- number of id switches     (IDs)
% [11]  FRA       - number of fragmentations
% [12]  MOTA	- Multi-object tracking accuracy in [0,100]
% [13]  MOTP	- Multi-object tracking precision in [0,100] (3D) / [td,100] (2D)
% [14]  MOTAL	- Multi-object tracking accuracy in [0,100] with log10(idswitches)
%

combinedMets = zeros(1,14);

MT=sum(allMets(:,5));
PT=sum(allMets(:,6));
ML=sum(allMets(:,7));

FP = sum(allMets(:,8));
FN = sum(allMets(:,9));
ID = sum(allMets(:,10));
FM = sum(allMets(:,11));

MOTA = 100 * (1 - (FP + FN + ID) / nGT);
TP=nGT-FN;
Rc=100*TP/nGT; 
Pr=100*TP/(FP+TP);
FAR=FP/Fgt;

MOTP=mean(allMets(:,13));
MOTAL=100 * (1 - (FP + FN + log(ID+1)) / nGT);

combinedMets=[Rc, Pr, FAR, nTR, MT, PT, ML, FP, FN, ID, FM, MOTA, MOTP, MOTAL];

% if options.eval3d
%     MOTP=(1-sum(sum(d))/sum(c)/td) * 100; % avg distance to [0,100]
% else
%     MOTP=sum(ious(ious>=td & ious<Inf))/sum(c) * 100; % avg ol
% end
% 
% MOTAL=(1-((sum(m)+sum(fp)+log10(sum(mme)+1))/sum(g)))*100;
% MOTA=(1-((sum(m)+sum(fp)+(sum(mme)))/sum(g)))*100;
% recall=sum(c)/sum(g)*100;
% precision=sum(c)/(sum(fp)+sum(c))*100;
% FAR=sum(fp)/Fgt;
 