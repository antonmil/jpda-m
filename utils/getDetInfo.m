function [detFolder, detFile]=getDetInfo(seqName,dataDir)
% get info about detection folder and file

detFolder = [dataDir,seqName,filesep,'det',filesep];
detFile = [detFolder,'det.txt'];
    
end