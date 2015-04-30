close all
clear all
% clc

addpath('C:\gurobi563\win64\matlab\');
addpath(genpath('clustering'))
pwdd= cd('../..');
pwds=[pwd,'\Codes\Bounding box estimation'];
pwds2=[pwd,'\Codes\motutils'];
pwdd2=[pwd,'\Data'];
cd(pwdd);
addpath(pwds)
addpath(genpath(pwds2))
Prun_Thre=0.3;

S_Name='S2';
L_Name='L2';
Image_address=[pwdd2,'\PETS\PETS09\Images\',S_Name,'\',L_Name];
Detection_address=[pwdd2,'\PETS\PETS09\Detections_and_GT\PETS09-',S_Name,L_Name];


file = dir(Image_address);
num_file = numel(file);

info = imfinfo([Image_address,'\',file(3).name]);
u_image=info(1).Height;
v_image=info(1).Width;
cl_image=class(imread([Image_address,'\',file(3).name]));

%% Load images and detections
I=zeros(u_image,v_image,3,num_file-2,cl_image);
load(Detection_address)

XYZ=cell(1,num_file-2);
% Detection_Evaluation

figure,
for k=1:num_file-2
    filename = strcat([Image_address,'\'],file(k+2).name);
    I(:,:,:,k) = imread(filename);
    %     XYZ{1,k}=[detections(k).bx+detections(k).wd/2;detections(k).by+detections(k).ht/2]';
    XYZ{1,k}=[detections(k).xi(detections(k).sc>Prun_Thre);detections(k).yi(detections(k).sc>Prun_Thre)]';
    
    
    imshow(I(:,:,:,k)),
    hold on
    X_D=detections(k).xi(detections(k).sc>Prun_Thre);
    Y_D=detections(k).yi(detections(k).sc>Prun_Thre);
    X_D2=detections(k).bx(detections(k).sc>Prun_Thre);
    Y_D2=detections(k).by(detections(k).sc>Prun_Thre);
    W_D2=detections(k).wd(detections(k).sc>Prun_Thre);
    H_D2=detections(k).ht(detections(k).sc>Prun_Thre);
    
    
    plot(X_D,Y_D,'og')
    hold on
    for jj=1:size(X_D,2)
        rectangle('Position',[X_D2(jj),Y_D2(jj),W_D2(jj),H_D2(jj)],'EdgeColor','g')
    end
    hold on
    X_F=detections(k).xi(detections(k).sc<=Prun_Thre);
    Y_F=detections(k).yi(detections(k).sc<=Prun_Thre);
    X_F2=detections(k).bx(detections(k).sc<=Prun_Thre);
    Y_F2=detections(k).by(detections(k).sc<=Prun_Thre);
    W_F2=detections(k).wd(detections(k).sc<=Prun_Thre);
    H_F2=detections(k).ht(detections(k).sc<=Prun_Thre);
    
    plot(X_F,Y_F,'or')
    hold on
    for jj=1:size(X_F,2)
        rectangle('Position',[X_F2(jj),Y_F2(jj),W_F2(jj),H_F2(jj)],'EdgeColor','r')
    end
    hold on
    idnxx=find(gtInfo.X(k,:)~=0);
    X_G=gtInfo.X(k,idnxx);
    Y_G=gtInfo.Y(k,idnxx);
    W_G=gtInfo.W(k,idnxx);
    H_G=gtInfo.H(k,idnxx);
    plot(X_G,Y_G,'+b')
    hold on
    for jj=1:size(X_G,2)
        rectangle('Position',[X_G(jj)-W_G(jj)/2,Y_G(jj)-H_G(jj),W_G(jj),H_G(jj)],'EdgeColor','b')
    end
    hold off
    
    title(num2str(k-1))
    %     pause(0.01)
end


%% Tracking Using IMM-JPDA

PD=0.89; % PD=0.95*ones(u,v,Frame);
T=1;% Temporal sampling rate
JPDA_P=[3/(u_image*v_image),30^0.5];%Beta=2/(u_image*v_image);Gate^2=30;
S_limit=100;


Initiation='TotalBased'; % The parameter for initiation


%Dynamic Matrices
% Model 1
F11=[1 T;0 1];
F(:,:,1)=blkdiag(F11,F11); % The transition matrix for the dynamic model 1
q1=0.5; % The standard deviation of the process noise for the dynamic model 1
Q11x=q1*[T^3/3 T^2/2;T^2/2 T];
Q11y=q1*[T^3/3 T^2/2;T^2/2 T];
Q(:,:,1)=blkdiag(Q11x,Q11y); % The process covariance matrix for the dynamic model 1


% Measurement Matrices
H=[1 0 0 0;0 0 1 0]; % % Measurement matrix
R=[7 0;0 7]; % Measurement covariance matrix

% JPDA Parameters
Tracking_Scheme='JPDA';
JPDA_multiscale=1; % Time-Frame windows
N_H=100; % Threshold for considering maximum number of Hypotheses

% JIPDA Parameters
Pdnt0=[0.65;0.009;0.341];
TPM_dnt=[0.59 0.49 0; 0.409 0.5 0];
Term_tre=[.6 0.7]; % The probabilities for track confirmation and termination


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
P0=blkdiag([7 0;0 1],[7 0;0 1]);

% visualization parameters
trk_plt='Yes';
vis_par=[10 20 100]; %[tail w_box h_box]

gtInfo2=gtInfo;
gtInfo2.Xgp=gtInfo.X;
gtInfo2.Ygp=gtInfo.Y;


tStart = tic;
[XeT,PeT,Xe,Pe,Ff,P_dnt_T,ten_cnf_ter,mui]=MULTISCAN_JIPDA(XYZ,F,Q,H,R,X02,P0,Tracking_Scheme,JPDA_P,N_H,...
    JPDA_multiscale,PD,S_limit,mui0,TPM,TPM_Option,H_TPM,Pdnt0,TPM_dnt,Term_tre,Initiation,I,trk_plt,vis_par);
tElapsed1_i = toc(tStart) %#ok<NOPTS>


%% Post-processing (post processing and Removing tracks with smal life spans)
conf_trk=cell2mat(cellfun(@(x) any(x==1), ten_cnf_ter, 'UniformOutput', false));
XeT=XeT(conf_trk);
Ff=Ff(conf_trk);
X_size=cellfun(@(x) size(x,2), XeT, 'UniformOutput', false);
Ff=cellfun(@(x,y) x(1):x(1)+y-1, Ff,X_size, 'ErrorHandler', @errorfun, ...
    'UniformOutput', false);
Ff_size=cellfun(@(x) size(x,2), Ff, 'UniformOutput', false);
XeT=cellfun(@(x,y) x(:,1:y),XeT,Ff_size, 'ErrorHandler', @errorfun, ...
    'UniformOutput', false);

% tret=15; % Remove Track with life time less than this threshold
% Ff(cellfun('size', XeT,2)<tret)=[];
% XeT(cellfun('size', XeT,2)<tret)=[];

%% Bounding box estimation from detection

cd('..');
addpath([pwd,'\Bounding box estimation'])
cd(pwdd);

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

if strcmp(S_Name,'S2')&&strcmp(L_Name,'L2')
    detections2=detections;
    for it=1:size(detections,2)
        detections2(it).xp=detections2(it).xi;
        detections2(it).yp=detections2(it).yi;
    end
else
    detections2=detections;
end

stateInfo=getBBoxesFromState(stateInfo,Ff,detections2,sceneInfo);
% save([pwd,'\Results\Tracking_results_',S_Name,L_Name],'I','XeT','Ff','stateInfo','sceneInfo')

%% Evaluation

cd('..');
addpath([pwd,'\MOT performance measure'])
addpath([pwd,'\Visualization'])
cd(pwdd);



stateInfo2=stateInfo;
stateInfo2.frameNums=stateInfo.frameNums-1;

% Evaluation all results using ground plan
options.eval3d=1;   % only bounding box overlap
options.td=1000;

[stateInfo2.Xgp,stateInfo2.Ygp]=projectToGroundPlane(stateInfo2.Xi, stateInfo2.Yi, sceneInfo);

[metrics,metricsInfo,additionalInfo]=CLEAR_MOT(gtInfo,stateInfo2,options);

printMetrics(metrics)

% Evaluation on cropped results using ground plan
stateInfo3=stateInfo2;
gtInfo3=gtInfo;

stateInfo3.X= stateInfo3.Xgp;
stateInfo3.Y= stateInfo3.Ygp;
gtInfo3.X=gtInfo3.Xgp;
gtInfo3.Y=gtInfo3.Ygp;
sceneInfo.trackingArea=[-14069.6, 4981.3, -14274.0, 1733.5];


gtInfoC=cutGTToTrackingArea(gtInfo3, sceneInfo);

opt.track3d=1;
stateInfoC=cutStateToTrackingArea(stateInfo3,sceneInfo, opt);

[metricsC,metricsInfoC,additionalInfoC]=CLEAR_MOT(gtInfoC,stateInfoC,struct('eval3d',1));

printMetrics(metricsC,metricsInfoC,1)

% Evaluation on cropped results using bounding box
stNew= stateInfoC;
gtNew=gtInfoC;

stNew.opt.track3d=0;

stNew.X=stNew.Xi;stNew.X=stNew.Yi;
gtNew.X=gtNew.Xi;gtNew.Y=gtNew.Yi;

[metrics3d, metricsInfo3d, addInfo3d]=CLEAR_MOT(gtNew,stNew,struct('eval3d',0));
printMetrics(metrics3d,metricsInfo3d,1);

%% Visualization

sceneInfo.imgFolder=[Image_address,'\'];
displayTrackingResult(sceneInfo, stateInfo2)








