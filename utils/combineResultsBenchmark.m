function mr=combineResultsBenchmark(jobname,maxexper)

if nargin<2
    maxexper=20;
end

mr=0;

format compact
% addpath(genpath('./motutils'));
resdir=sprintf('results/%s',jobname);
disp(resdir)
resfiles=dir(sprintf('%s/res_*.mat',resdir));


%  dfile=[resdir filesep 'bestres.txt'];
%  if exist(dfile,'file'), delete(dfile); end
% diary(dfile);
% fid=fopen(dfile,'w');


if isempty(resfiles)
    warning('nothing there yet!\n')
    return;
end

eval3d=0;


load(sprintf('%s/%s',resdir,resfiles(1).name));
%  allscen=find(mets2d(:,1))';





% allscen=allscen(1:3);
% maxexper=3;
% global gtInfo

% concat gtInfo
gtInfoAll=[];
gtInfoAll.Xi=[];
allFgt=zeros(1,length(allscen));

    allscenes={
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
    

% Find out the length of each sequence
% and concatenate ground truth
gtInfoSingle=[];
seqCnt=0;
fprintf('Concatenate ground truth\n');
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


%% for all results
allmets=[];
for r=1:maxexper
    fprintf('Experiment: %d\n',r);
    resfile=sprintf('%s/res_%03d.mat',resdir,r);
    if exist(resfile,'file')
        load(resfile);
        % allscen=allscen(1:3);
        
        allMetsAnton=[];
        
        clear stInfo
        stInfo.Xi=[];
        
        evalMethod=1;
        
        seqCnt=0;
        trackerRuntime=0;
        
        % ... and each sequence
        allscen=1:length(allscenes);
        for scen=allscen
            seqCnt=seqCnt+1;
%             sceneInfo=getSceneInfo(scen);
            
%             sequence = sceneInfo.sequence;
            sequence = char(allscenes(scen));
            fprintf('\t... %s\n',sequence);
            
            stI = infos(scen).stateInfo;
            [FI,NI] = size(stI.Xi);
            % check if bounding boxes available in solution
            imCoord=1;
            if NI>0 && all(stI.Xi(find(stI.Xi(:)))==-1)
                imCoord=0;
            end
            worldCoordST=0; % state
            if isfield(stI,'Xgp') && isfield(stI,'Ygp')
                worldCoordST=1;
            end
            
            % if stateInfo shorter, pad with zeros
            % GT and result must be equal length
            if FI<allFgt(seqCnt)
                missingFrames = FI+1:allFgt(seqCnt);
                stI.Xi(missingFrames,:)=0;
                stI.Yi(missingFrames,:)=0;
                stI.W(missingFrames,:)=0;
                stI.H(missingFrames,:)=0;
                stI.X(missingFrames,:)=0;
                stI.Y(missingFrames,:)=0;
                if worldCoordST
                    stI.Xgp(missingFrames,:)=0; stI.Ygp(missingFrames,:)=0;
                end
                
                [FI,NI] = size(stI.Xi);
            end
            allMetsAnton.mets2d(seqCnt).name=sequence;
            allMetsAnton.mets3d(seqCnt).name=sequence;
            
            % if world coordinates available and challenge is 3d, evaluate in 3D
            if  gtInfoSingle(seqCnt).wc && worldCoordST && eval3d
                evopt.eval3d=1;evopt.td=1;
                [mets, mInf]=CLEAR_MOT_HUN(gtInfoSingle(seqCnt).gtInfo,stI,evopt);
                allMetsAnton.mets3d(seqCnt).m=mets;
                fprintf('*** 3D (in world coordinates) ***\n'); printMetrics(mets); fprintf('\n');
                % diary(dfile); printMetrics(mets); fprintf('\n');diary off
                
            elseif imCoord && ~eval3d
                [mets, mInf]=CLEAR_MOT_HUN(gtInfoSingle(seqCnt).gtInfo,stI);
                allMetsAnton.mets2d(seqCnt).m=mets;
                fprintf('*** 2D (Bounding Box overlap) ***\n'); printMetrics(mets); fprintf('\n');
                % diary(dfile); printMetrics(mets); fprintf('\n');diary off
                
            else
                fprintf('WARNING: Run %d not complete, results corrupt?\n', r);
                evalMethod = 0;
                break;
            end
            [F,N] = size(stInfo.Xi);
            newF = F+1:F+FI;
            newN = N+1:N+NI;
            
            % concat result
            stInfo.Xi(newF,newN) = stI.Xi;
            stInfo.Yi(newF,newN) = stI.Yi;
            stInfo.W(newF,newN) = stI.W;
            stInfo.H(newF,newN) = stI.H;
            
            if isfield(stI,'Xgp') && isfield(stI,'Ygp')
                stInfo.Xgp(newF,newN) = stI.Xgp;stInfo.Ygp(newF,newN) = stI.Ygp;
            end
            stInfo.X=stInfo.Xi;stInfo.Y=stInfo.Yi;
            if eval3d
                stInfo.X=stInfo.Xgp;stInfo.Y=stInfo.Ygp;
            end
            
        end
        
        stInfo.frameNums=1:size(stInfo.Xi,1);
        
        if evalMethod
            if eval3d
                evopt.eval3d=1;evopt.td=1;
                [mets, mInf]=CLEAR_MOT_HUN(gtInfoAll,stInfo,evopt);
                fprintf('*** 3D (in world coordinates) ***\n'); printMetrics(mets); fprintf('\n');
            else
                gtInfoAll
                stInfo
                [mets, mInf]=CLEAR_MOT_HUN(gtInfoAll,stInfo);
                fprintf('*** 2D (Bounding Box overlap) ***\n'); printMetrics(mets); fprintf('\n');
            end
            
            evalFile = fullfile(resdir, 'eval_anton.txt');
            dlmwrite(evalFile,mets);
            
            
            
        else
            fprintf('WARNING: Run %d cannot be evaluated\n',r);
        end
        
        fprintf('\t ...done\n')
        allmets(r,:)=mets;
        
    end
end


diary off;
dfile=[resdir filesep 'bestres.txt'];
if exist(dfile,'file'), delete(dfile); end
diary(dfile);

fprintf('done %d experiments\n',length(resfiles));

% best MOTA
allmota=allmets(:,12)';
fprintf('%4d  ',1:maxexper); fprintf('\n');
fprintf('%4.1f  ',allmota); fprintf('\n');
[mm, mr]=max(allmota);


% load best
load(sprintf('%s/res_%03d.mat',resdir,mr));
fprintf('Best: %d\n',mr);

%% write a text file

bestmets=[];
for s=allscen

    tmp=mets2d(s,:);
    printMetrics(tmp);
    bestmets=[bestmets; tmp];
end

fprintf('--------------------------------------------------------\n');
intmets=4:11;
meanmets=mean(bestmets,1);
meanmets(:,intmets)=round(meanmets(:,intmets));

printMetrics(allmets(mr,:));
diary off


