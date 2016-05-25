%% Run once on training
function allMets = runTraining(conffile)
resDir='res/test';
logDir='logs';
if ~exist(logDir,'dir'), mkdir(logDir); end
logFile=[logDir,filesep,sprintf('test.log')];
delete(logFile); %delete log file if it exists
diary(logFile);

% find out best result
% [bestMOTA, bestRUN]=max(allres(:,12));
% 
% fprintf('best MOTA %f at run %d\n',bestMOTA,bestRUN);
% fprintf('Running TEST\n');

%%%% run test %%%
% make a copy of best training run config
% bestconf=sprintf('config/%04d.ini',bestRUN);
% copyfile(bestconf,'config/test.ini');
% runTracker([1 challenge],'config/test.ini',resDir);
allseq={
    'TUD-Stadtmitte', ...
    'TUD-Campus', ...
    'PETS09-S2L1', ...
    'ETH-Bahnhof', ...
    'ETH-Sunnyday', ...
    'ETH-Pedcross2', ...
    'ADL-Rundle-6', ...
    'ADL-Rundle-8', ...
    'KITTI-13', ...
    'KITTI-17', ...
    'Venice-2', ...
    };

if nargin<1
    conffile='config/default.ini';
end

for seq=allseq
    [m2d,m3d,stateInfo]=runMFJPDA(char(seq),conffile,resDir);
end

%%
structMets = evaluateTracking('c2-train.txt',resDir,getDataDir);
allMets = structMets.bmark2d;

dlmwrite([resDir,'/allres.txt'],allMets);
