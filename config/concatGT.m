function [gtInfoAll,gtInfoSingle]=concatGT(allscenes, eval3d)

gtInfoSingle=[];
gtInfoAll=[];
gtInfoAll.Xi=[];

seqCnt=0;

for scen=allscenes
    seqCnt=seqCnt+1;
%     sceneInfo=getSceneInfo(scen);
    
%     sequence = sceneInfo.sequence;
    sequence = char(scen);
    fprintf('\t... %s\n',sequence);
%     gtI = gtInfo;

    [seqName, seqFolder, imgFolder, imgExt, seqLength, dirImages] = ...
        getSeqInfo(sequence, getDataDir);

     gtFile=[seqFolder,filesep,'gt/gt.txt'];
%     gtI = convertTXTToStruct(gtFile,seqFolder);
    
        gtInfo=convertTXTToStruct(gtFile);
        Fgt=seqLength;
        FE=size(gtInfo.W,1);
        % if stateInfo shorter, pad with zeros
        if FE<Fgt
            missingFrames = FE+1:Fgt;
            gtInfo.Xi(missingFrames,:)=0;
            gtInfo.Yi(missingFrames,:)=0;
            gtInfo.W(missingFrames,:)=0;
            gtInfo.H(missingFrames,:)=0;
			gtInfo.X(missingFrames,:)=0;
			gtInfo.Y(missingFrames,:)=0;
        end
        gtInfo.frameNums=1:Fgt;
        gtI = gtInfo;
        
    [Fgt,Ngt] = size(gtInfoAll.Xi);
    [FgtI,NgtI] = size(gtI.Xi);
    newFgt = Fgt+1:Fgt+FgtI;
    newNgt = Ngt+1:Ngt+NgtI;
    
    gtInfoAll.Xi(newFgt,newNgt) = gtI.Xi;
    gtInfoAll.Yi(newFgt,newNgt) = gtI.Yi;
    gtInfoAll.W(newFgt,newNgt) = gtI.W;
    gtInfoAll.H(newFgt,newNgt) = gtI.H;
    
    gtInfoSingle(seqCnt).wc=0;
    
    % fill in world coordinates if they exist
    if isfield(gtI,'Xgp') && isfield(gtI,'Ygp')
        gtInfoAll.Xgp(newFgt,newNgt) = gtI.Xgp;
        gtInfoAll.Ygp(newFgt,newNgt) = gtI.Ygp;
        gtInfoSingle(seqCnt).wc=1;
    end
    
    % check if bounding boxes available in gt
    imCoord=1;
    if all(gtI.Xi(find(gtI.Xi(:)))==-1)
        imCoord=0;
    end
    
    gtInfoAll.X=gtInfoAll.Xi;gtInfoAll.Y=gtInfoAll.Yi;
    if eval3d
        gtInfoAll.X=gtInfoAll.Xgp;gtInfoAll.Y=gtInfoAll.Ygp;
    end
    
    allFgt(seqCnt) = FgtI;
    
    gtInfoSingle(seqCnt).gtInfo=gtI;
    
end
gtInfoAll.frameNums=1:size(gtInfoAll.Xi,1);