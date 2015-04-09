function [seqName, seqFolder, imgFolder, imgExt, F, dirImages] ...
    = getSeqInfo(seq, dataDir)
% construct variables with relevant information about the sequence 'seq'
% 

    seqName=char(seq)
    seqFolder= [dataDir,seqName,filesep];
    
    imgFolders = dir(fullfile(seqFolder,filesep,'img*'));   
    imgFolder = [seqFolder,imgFolders(1).name,filesep];
    imgExt = getImgExt(seqFolder);
    imgMask=[imgFolder,'*', imgExt];
    dirImages = dir(imgMask);
    F=length(dirImages);
    
end
