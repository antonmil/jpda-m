function [metrics2d, metrics3d, stateInfo]=runMFJPDA(sequence,options,outdir)

% close all
% clear all
% clc

metrics2d=zeros(1,14);
metrics3d=zeros(1,14);
stateInfo=[];


% addpath(genpath('/home/amilan/research/projects/bmtt-dev/scripts')) % tools
trackerName = 'MFJPDA';
addpath(genpath('.'));

resDir = fullfile(getResDir(),trackerName,'data',filesep);


if ~nargin, sequence=[]; options='config/default.ini'; outdir=resDir; end
if nargin==1, options='config/default.ini'; outdir=resDir; end
if nargin==2, outdir=resDir; end
options = parseOptions(options);

dataDir = getDataDir();
allseq = parseSequences(sequence,dataDir);


if ~exist(outdir,'dir'), mkdir(outdir); end

% runtime
rtDir = fullfile(getResDir(),trackerName,'runtimes',filesep);
if ~exist(rtDir,'dir'), mkdir(rtDir); end


Prun_Thre=30; % det threshold
% Prun_Thre=.99999; % det threshold

tret=15; % Remove Track with life time less than this threshold
Term_Frame=15; % The parameter for termination condition

PD=0.89; % PD=0.95*ones(u,v,Frame);
q1=0.5; % The standard deviation of the process noise for the dynamic model 1

Mcov=7;



% addpath('C:\gurobi563\win64\matlab\');
% addpath('/home/amilan/software/gurobi603/linux64/matlab/');
% addpath(genpath('clustering'))
% addpath('motutils');
% addpath('Bounding box estimation');

% pwdd= cd('../..');
% pwds=[pwd,'\Codes\Bounding box estimation'];
% pwds2=[pwd,'\Codes\motutils'];
% pwdd2=[pwd,'\Data'];
%
% cd(pwdd);
% addpath(pwds)
% addpath(genpath(pwds2))

%
% S_Name='S2';
% L_Name='L2';
% Image_address=[pwdd2,'\PETS\PETS09\Images\',S_Name,filesep,L_Name];
% Detection_address=[pwdd2,'\PETS\PETS09\Detections_and_GT\PETS09-',S_Name,L_Name];

% seqFolder='/home/amilan/research/projects/bmtt-data/PETS09-S2L2';
% seqFolder='/home/amilan/research/projects/bmtt-data/TUD-Stadtmitte';
% seqFolder='/home/amilan/research/projects/bmtt-data/ETH-Bahnhof';
% seqFolder='/home/amilan/research/projects/bmtt-data/ETH-Sunnyday';
% seqFolder='/home/amilan/research/projects/bmtt-data/ADL-Rundle-6';
% seqFolder='/home/amilan/research/projects/bmtt-data/ADL-Rundle-8';
% seqFolder='/home/amilan/research/projects/bmtt-data/KITTI-13';
% seqFolder='/home/amilan/research/projects/bmtt-data/KITTI-17';

nSeq=length(allseq);
% doparallel=0;
% 
% if nSeq>2
%     doparallel=min(7,nSeq);
% end


% parpool(7)

% parfor ns=1:nSeq
for ns=1:nSeq
    seq = allseq(ns);

    [seqName, seqFolder, imgFolder, imgExt, seqLength, dirImages] = ...
        getSeqInfo(seq, dataDir);
    
    fprintf('MF JPDA: %s\n',seqName);
    %%

    fR=getFrameRate(seqName);
    param=setOptions(options,fR); % set custom params

    Prun_Thre = param.Prun_Thre;
    tret = param.tret;
    Term_Frame = param.Term_Frame;
    PD = param.PD;
    q1 = param.q1;
    Mcov = param.Mcov;



    %%%%
    Gatesq = param.Gatesq;
    FPPI = param.FPPI;
    Upos=param.Upos; % uncertainty in initial position
    Uvel=param.Uvel; % uncertainty in initial velocity
    T=1;% Temporal sampling rate    
    S_limit=100; % complexity, controls gating size
    JPDA_multiscale=param.MF; % Time-Frame windows
    N_H=param.m; % Threshold for considering maximum number of Hypotheses (m best)
    AR=param.AR;
    fpn=param.fpn;
    

    % 1-Prun_Thre
    disp([Prun_Thre,tret,Term_Frame,PD,q1,Mcov,Gatesq,FPPI,Upos,Uvel,AR,fpn])
    
    Image_address=[seqFolder,filesep,'img1'];
    file = dir([Image_address,filesep,'*.jpg']);
    num_file = numel(file)
    % F = num_file;
    
    info = imfinfo([Image_address,filesep,file(3).name]);
    u_image=info(1).Height;
    v_image=info(1).Width;
    cl_image=class(imread([Image_address,filesep,file(3).name]));
    
    %%
    GT_address=[seqFolder,filesep,'gt/gt.txt'];
    Detection_address=[seqFolder,filesep,'det/det.txt'];

    
    
    
    fprintf('Loading detections...');
    [detFolder, detFile]=getDetInfo(seqName,dataDir);
    detRaw=dlmread(detFile);
    
    % super quick and dirty hack
    if (strcmp(seqName,'ETH-Pedcross2'))
        detRaw=[1,-1,155,265,26,59,2.0863,-1,-1,-1; detRaw];
    end
    
    sceneInfo=[];
    sceneInfo.targetSize = mean(detRaw(:,5))/2;
    sceneInfo.targetAR = mean(detRaw(:,5)./detRaw(:,6));
%     sceneInfo.targetAR = 0.45;
    sceneInfo.targetAR = param.AR;
    sceneInfo.fpn = param.fpn;

    sceneInfo.gtAvailable=0;
    sceneInfo.imgHeight = u_image;sceneInfo.imgWidth = v_image;
    sceneInfo.imgFileFormat='%06d.jpg';
    sceneInfo.imgFolder=[Image_address,filesep];
    sceneInfo.frameNums = 1:num_file;
    frameRateFile=[seqFolder,filesep,'framerate.txt'];
    try sceneInfo.frameRate = dlmread(frameRateFile);
    catch err
        sceneInfo.frameRate = 25; fprintf('Could not determine frame rate. Setting to 25 FPS!\n');
    end
    
    Gatesq = sceneInfo.targetSize*3;
%     percentile=0.25;
%     allsc=sort(detRaw(:,7));
%     Prun_Thre=allsc(round(percentile*length(allsc)));
%     Prun_Thre=median(detRaw(:,7));
%      disp(Prun_Thre);
    fprintf('< %f',Prun_Thre);
    
    detections=[];
    for t=1:num_file
        detections(t).bx=[];
        detections(t).by=[];
        detections(t).xi=[];
        detections(t).yi=[];
        detections(t).xp=[];
        detections(t).yp=[];
        detections(t).wd=[];
        detections(t).ht=[];
        detections(t).sc=[];
    end
        
    for d=1:size(detRaw,1)
        t=detRaw(d,1);
        
        w=detRaw(d,5);
        h=detRaw(d,6);
        bx=detRaw(d,3);
        by=detRaw(d,4);
        xi=detRaw(d,3)+w/2;
        yi=detRaw(d,4)+h;
        sc=detRaw(d,7);
%         sc(:)=1./(1+exp(-sc));
        % SCORES?

        
        detections(t).bx=[detections(t).bx bx];
        detections(t).by=[detections(t).by by];
        detections(t).xi=[detections(t).xi xi];
        detections(t).yi=[detections(t).yi yi];
        detections(t).xp=[detections(t).xp xi];
        detections(t).yp=[detections(t).yp yi];
        detections(t).wd=[detections(t).wd w];
        detections(t).ht=[detections(t).ht h];
        detections(t).sc=[detections(t).sc sc];
    end
    fprintf('... %d detections present.',numel([detections(:).sc]));
    
    
    
    
    
    
    
    
    
    %% Load images and detections
    I=zeros(u_image,v_image,3,num_file,cl_image);
    % load(Detection_address)
    
    XYZ=cell(1,num_file);
    % Detection_Evaluation
    
    showdets=0;
    
    ndet=0;
    for k=1:num_file
        %     XYZ{1,k}=[detections(k).bx+detections(k).wd/2;detections(k).by+detections(k).ht/2]';
        XYZ{1,k}=[detections(k).xi(detections(k).sc>Prun_Thre);detections(k).yi(detections(k).sc>Prun_Thre)]';
        ndet=ndet+numel(find(detections(k).sc>Prun_Thre));
        
        
        X_D=detections(k).xi(detections(k).sc>Prun_Thre);
        Y_D=detections(k).yi(detections(k).sc>Prun_Thre);
        X_D2=detections(k).bx(detections(k).sc>Prun_Thre);
        Y_D2=detections(k).by(detections(k).sc>Prun_Thre);
        W_D2=detections(k).wd(detections(k).sc>Prun_Thre);
        H_D2=detections(k).ht(detections(k).sc>Prun_Thre);
        
        if showdets
            filename = strcat([Image_address,filesep],file(k+2).name);
            I(:,:,:,k) = imread(filename);
            
            figure,
            imshow(I(:,:,:,k)),
            hold on
            
            plot(X_D,Y_D,'og')
            hold on
            for jj=1:size(X_D,2)
                rectangle('Position',[X_D2(jj),Y_D2(jj),W_D2(jj),H_D2(jj)],'EdgeColor','g')
            end
            hold on
        end
        
        X_F=detections(k).xi(detections(k).sc<=Prun_Thre);
        Y_F=detections(k).yi(detections(k).sc<=Prun_Thre);
        X_F2=detections(k).bx(detections(k).sc<=Prun_Thre);
        Y_F2=detections(k).by(detections(k).sc<=Prun_Thre);
        W_F2=detections(k).wd(detections(k).sc<=Prun_Thre);
        H_F2=detections(k).ht(detections(k).sc<=Prun_Thre);
        
        if showdets
            plot(X_F,Y_F,'or')
            hold on
            for jj=1:size(X_F,2)
                rectangle('Position',[X_F2(jj),Y_F2(jj),W_F2(jj),H_F2(jj)],'EdgeColor','r')
            end
            title(num2str(k-1))
            drawnow
        end
        %     hold on
        %     idnxx=find(gtInfo.X(k,:)~=0);
        %     X_G=gtInfo.X(k,idnxx);
        %     Y_G=gtInfo.Y(k,idnxx);
        %     W_G=gtInfo.W(k,idnxx);
        %     H_G=gtInfo.H(k,idnxx);
        %     plot(X_G,Y_G,'+b')
        %     hold on
        %     for jj=1:size(X_G,2)
        %         rectangle('Position',[X_G(jj)-W_G(jj)/2,Y_G(jj)-H_G(jj),W_G(jj),H_G(jj)],'EdgeColor','b')
        %     end
        %     hold off
        
        
        %     pause(0.01)
    end
    fprintf('... %d detections present after pruning\n',ndet);
    
    %%%%%%% START
    runtime=tic;
    
    % pause
    %% Tracking Using IMM-JPDA
    
    
    JPDA_P=[FPPI/(u_image*v_image),Gatesq];%Beta=2/(u_image*v_image);Gate^2=30; 3 = FPPI
    
    Initiation='TotalBased'; % The parameter for initiation
    
    
    %Dynamic Matrices
    % Model 1
    F11=[1 T;0 1];
    F=[]; Q=[];
    F(:,:,1)=blkdiag(F11,F11); % The transition matrix for the dynamic model 1
    Q11x=q1*[T^3/3 T^2/2;T^2/2 T];
    Q11y=q1*[T^3/3 T^2/2;T^2/2 T];
    Q(:,:,1)=blkdiag(Q11x,Q11y); % The process covariance matrix for the dynamic model 1
    
    
    % Measurement Matrices
    H=[1 0 0 0;0 0 1 0]; % % Measurement matrix
    R=[Mcov 0;0 Mcov]; % Measurement covariance matrix
    
    % JPDA Parameters
    Tracking_Scheme='JPDA';
%     JPDA_multiscale=1; % Time-Frame windows
%     N_H=100; % Threshold for considering maximum number of Hypotheses
    
    
    % PD Parameters
    PD_Option='Constant'; % PD_Option='State-Dependent';
    H_PD=[1 0 0 0;0 0 1 0];
    
    % IMM_Parameters
    mui0=1; % The initial values for the IMM probability weights
    TPM_Option='Constant'; % TPM_Option='State-Dependent';
    TPM=1; % TPM{2,2}=1;TPM{2,1}=2*eye(2,2);TPM{1,2}=1;TPM{1,1}=0.5*eye(2,2);
    H_TPM=[0 1 0 0;0 0 0 1];
    
    % The input measurements for the all sequences
    DSE=2;DMV=size(H,1);
    
    % The initial state distribution parameters
    Vmax=7;
    X02=Initialization(XYZ,DSE,T,Vmax);
    P0=blkdiag([Upos 0;0 Uvel],[Upos 0;0 Uvel]);
    
    % gtInfo2=gtInfo;
    % gtInfo2.Xgp=gtInfo.X;
    % gtInfo2.Ygp=gtInfo.Y;
    
    
    tStart = tic;
    [XeT,PeT,Xe,Pe,Ff,Term_Con,mui]=MULTISCAN_JPDA(XYZ,F,Q,H,R,X02,P0,Tracking_Scheme,JPDA_P,N_H,...
        JPDA_multiscale,PD,S_limit,mui0,TPM,TPM_Option,H_TPM,Term_Frame,Initiation,I,'No',100);
%     tElapsed1_i = toc(tStart) %#ok<NOPTS>
    fprintf(1,'All done (%.2f min = %.2fh = %.2f SPF = %.2f FPS)\n',toc(tStart)/60,toc(tStart)/3600,toc(tStart)/num_file,num_file/toc(tStart));
    
    % [XeT,PeT,Xe,Pe,Ff,Term_Con,mui]=MULTISCAN_JPDA_CS(XYZ,WHP,F,Q,H,R,X0,P0,Tracking_Scheme,JPDA_P,N_H,...
    %     JPDA_multiscale,PD,S_limit,mui0,TPM,TPM_Option,H_TPM,Term_Frame,Initiation,I,'No',100);
    
    
    %% Post-processing (post processing and Removing tracks with smal life spans)
    
    X_size=cellfun(@(x) size(x,2), XeT, 'UniformOutput', false);
    Ff=cellfun(@(x,y,z) x(1):x(1)+y-1-z, Ff,X_size,Term_Con, 'ErrorHandler', @errorfun, ...
        'UniformOutput', false);
    Ff_size=cellfun(@(x) size(x,2), Ff, 'UniformOutput', false);
    XeT=cellfun(@(x,y) x(:,1:y),XeT,Ff_size, 'ErrorHandler', @errorfun, ...
        'UniformOutput', false);
    XeT2=XeT;
    Ff2=Ff;
    Ff(cellfun('size', XeT,2)<tret)=[];
    XeT(cellfun('size', XeT,2)<tret)=[];
    
    
    %% Bounding box estimation from detection
    
    % cd('..');
    % addpath([pwd,'\Bounding box estimation'])
    % cd(pwdd);
    
    Frame=size(XYZ,2);
    N_T=size(XeT,2);
    
    stateInfo.X=zeros(Frame,N_T);
    stateInfo.Y=zeros(Frame,N_T);
    stateInfo.Xi=zeros(Frame,N_T);
    stateInfo.Yi=zeros(Frame,N_T);
    
    for n=1:N_T
        stateInfo.X(Ff{n},n)=XeT{n}(1,:);
        stateInfo.Xi(Ff{n},n)=stateInfo.X(Ff{n},n);
        stateInfo.Y(Ff{n},n)=XeT{n}(3,:);
        stateInfo.Yi(Ff{n},n)=stateInfo.Y(Ff{n},n);
    end
    
    stateInfo.frameNums=1:Frame;
    
    % if strcmp(S_Name,'S2')&&strcmp(L_Name,'L2')
    %     detections2=detections;
    %     for it=1:size(detections,2)
    %         detections2(it).xp=detections2(it).xi;
    %         detections2(it).yp=detections2(it).yi;
    %     end
    % else
    %     detections2=detections;
    % end
    
    detections2=detections;
    
    
    sceneInfo.camFile=[];
    
    if exist('/home/amilan','dir')
%       if strcmp(seqName,'TUD-Stadtmitte')
% 	  sceneInfo.camFile='/home/amilan/storage/databases/TUD/tud-stadtmitte-calib.xml';
%       elseif ~isempty(strfind(seqName,'PETS'))
% 	  sceneInfo.camFile='/home/amilan/storage/databases/PETS2009/View_001.xml';
%       end
%       
%       if ~isempty(sceneInfo.camFile)        
% 	  sceneInfo.camPar=parseCameraParameters(sceneInfo.camFile);
% 	  [stateInfo.Xgp,stateInfo.Ygp]=projectToGroundPlane(stateInfo.Xi, stateInfo.Yi, sceneInfo);
% 	  stateInfo.Xgp=stateInfo.Xgp./1000;         stateInfo.Ygp=stateInfo.Ygp./1000;
%       end
%       
%       if (~isempty(strfind(seqName,'TownCentre')))
% 	  tmpRes='tmp.txt';
% 	  convertSTInfoToTXT(stateInfo, tmpRes);
%   %         tmpState = dlmread('tmp.txt');
% 	  Frames=readFramesLaura(tmpRes);
% 	      namefile='maps/TownCentre-calibration.ci';
% 	  Frames=calib_towncentre(Frames,namefile);
% 	  FromFrames2TXT(Frames,tmpRes); 
% 	  stateInfo=convertTXTToStruct(tmpRes,seqFolder);
%       end
    end
    
    
    
    stateInfo=getBBoxesFromState(stateInfo,Ff,detections2,sceneInfo);
    % save([pwd,'\Results\Tracking_results_',S_Name,L_Name],'I','XeT','Ff','stateInfo','sceneInfo')
    
    %%%%%% END
    runtime=toc(runtime);
    

    % write runtime
    rtFile = [rtDir,seqName,'.txt'];
    dlmwrite(rtFile,runtime);
    
    % write results
    resFile = fullfile(outdir,[seqName,'.txt']);

%     all_mot
    convertSTInfoToTXT(stateInfo, resFile);
%     writeResults(stateInfo,resFile);

    metrics2d=zeros(1,14);
    metrics3d=zeros(1,14);

    


    gtFile = fullfile(GT_address);

    if exist(gtFile,'file');
        evoptions=[];

        gtInfo = convertTXTToStruct(gtFile,seqFolder);
        gtInfo.frameNums=1:size(gtInfo.Xi,1);


        disp(seqName)
        evoptions.eval3d=0;   % only bounding box overlap
        evoptions.td=0.5;
        
        [metrics2d,~,~]=CLEAR_MOT_HUN(gtInfo,stateInfo,evoptions);
        fprintf('*** 2D (Bounding Box overlap) ***\n'); printMetrics(metrics2d); fprintf('\n');        
        
        if ~isempty(sceneInfo.camFile)   
            evoptions.eval3d=1; evoptions.td=1;        
            [metrics3d,~,~]=CLEAR_MOT_HUN(gtInfo,stateInfo,evoptions);
            fprintf('*** 3D (in world coordinates) ***\n'); printMetrics(metrics3d); fprintf('\n');
            
        end
        
%         printMetrics(metrics2d)
    end

    stateInfo.opt.track3d=0; stateInfo.opt.cutToTA=0;
    stateInfo.sceneInfo=sceneInfo;
    
end
% delete(gcp)
