%% Run once on TEST
function runTest(conffile)
resDir='res/test';
logDir='logs';
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
    'TUD-Crossing',...
    'PETS09-S2L2',...
    'ETH-Jelmoli',...
    'ETH-Linthescher',...
    'ETH-Crossing',...
    'AVG-TownCentre',...
    'ADL-Rundle-1',...
    'ADL-Rundle-3',...
    'KITTI-16',...
    'KITTI-19',...
    'Venice-1',...
    };

if nargin<1
    conffile='config/default.ini';
end

for seq=allseq
    [m2d,m3d,stateInfo]=runMFJPDA(char(seq),conffile,resDir);
end

%%
structMets = evaluateTracking('c2-test.txt',resDir,getDataDir);
allMets = structMets.bmark2d;

dlmwrite([resDir,'/allres.txt'],allMets);
