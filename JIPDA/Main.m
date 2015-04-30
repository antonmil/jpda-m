clear all
close all
addpath('C:\gurobi563\win64\matlab')
gurobi_setup
clc
pwdd= cd('../../..');
pwdd2=[pwd,'\Data'];
cd(pwdd);


Data_type='\SyncDumpRound14-135-1500';

Seq_name='R14Q3C1_output';
load([pwdd2,Data_type,'\Detections\',Seq_name,'.mat'])
Frame=size(detections,2);

Seq_name='1';
Image_address=[pwdd2,Data_type,'\Images\',Seq_name];
file = dir(Image_address);
num_file = numel(file);

info = imfinfo([Image_address,'\',file(3).name]);
u_image=info(1).Height;
v_image=info(1).Width;
cl_image=class(imread([Image_address,'\',file(3).name]));
SRt=1;

XYZ=cell(1,Frame);
I=zeros(u_image,v_image,3,Frame,cl_image);

for k=1:Frame
    XYZ{k}=[detections(k).xc detections(k).yc];
      filename = strcat([Image_address,'\'],file((k-1)*SRt+3).name);
    I(:,:,:,k)= imread(filename); 
    imshow(I(:,:,:,k))
    hold on
    for jj=1:size(detections(k).xc,1)
    rectangle('Position',[detections(k).xc(jj)-detections(k).w(jj)/2,...
        detections(k).yc(jj)-detections(k).h(jj)/2,...
        detections(k).w(jj),detections(k).h(jj)])
    end
    hold off
    pause(0.01)
end
    
%% Tracking parameters

PD=0.98; 
T=1;% Temporal sampling rate
JPDA_P=[3/(u_image*v_image),6];%Beta=2/(u_image*v_image);Gate=6;
S_limit=inf;


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
Pdnt0=[0.9;0.019;0.081];
TPM_dnt=[0.9 0.49 0; 0.08 0.5 0];
Term_tre=[.3 0.99]; % The probabilities for track confirmation and termination

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
vis_par=[10 20 50]; %[tail w_box h_box]



[XeT,PeT,Xe,Pe,Ff,P_dnt_T,mui]=MULTISCAN_JIPDA(XYZ,F,Q,H,R,X02,P0,Tracking_Scheme,JPDA_P,N_H,...
    JPDA_multiscale,PD,S_limit,mui0,TPM,TPM_Option,H_TPM,Pdnt0,TPM_dnt,Term_tre,Initiation,I,trk_plt,vis_par);

% [XeT,PeT,Xe,Pe,Ff,Term_Con,mui]=MULTISCAN_JPDA_AFL(XYZ,F,Q,H,R,X02,P0,Tracking_Scheme,JPDA_P,N_H,...
%     JPDA_multiscale,PD,S_limit,mui0,TPM,TPM_Option,H_TPM,Term_Frame,Initiation,I,'No',100);


%% Post-processing (post processing and Removing tracks with smal life spans)

X_size=cellfun(@(x) size(x,2), XeT, 'UniformOutput', false);
Ff=cellfun(@(x,y) x(1):x(1)+y-1, Ff,X_size, 'ErrorHandler', @errorfun, ...
    'UniformOutput', false);
Ff_size=cellfun(@(x) size(x,2), Ff, 'UniformOutput', false);
XeT=cellfun(@(x,y) x(:,1:y),XeT,Ff_size, 'ErrorHandler', @errorfun, ...
    'UniformOutput', false);
XeT2=XeT;
Ff2=Ff;

%% Visualization
N_T=size(XeT,2);
Frame=size(XYZ,2);
stateInfo.X=zeros(Frame,N_T);
stateInfo.Y=zeros(Frame,N_T);
stateInfo.Xi=zeros(Frame,N_T);
stateInfo.Yi=zeros(Frame,N_T);

for n=1:N_T
    stateInfo.X(Ff{n},n)=XeT{n}(1,:);
    stateInfo.Xi=stateInfo.X;
    stateInfo.Y(Ff{n},n)=XeT{n}(3,:);
    stateInfo.Yi=stateInfo.Y;
end

stateInfo.frameNums=1:Frame;

figure,
l_t=10;
colorord=.25+.75*rand(N_T,3);
for k=1:Frame
     filename = strcat([Image_address,'\'],file((k-1)*SRt+3).name);
    I= imread(filename);
    imshow(I),
    hold on
    for n=1:N_T
        ixt=sort(find(stateInfo.X(:,n)~=0));
        tim1=max(k-l_t,ixt(1));
        if (stateInfo.X(k,n))
            rectangle('Position',[stateInfo.X(k,n)-20/2,stateInfo.Y(k,n)-50/2,20,50],'EdgeColor',colorord(n,:),'LineWidth',1)
            
            % rectangle('Position',[stateInfo.X(k,n)-stateInfo.W(k,n)/2,stateInfo.Y(k,n)-stateInfo.H(k,n),stateInfo.W(k,n),stateInfo.H(k,n)],'EdgeColor',colorord(n,:))
            text(stateInfo.X(k,n),stateInfo.Y(k,n),num2str(n),'Color',colorord(n,:))
            %          plot(XeT{1,n}(1,idx),XeT{1,n}(3,idx),'o','Color',colorord(n,:),'MarkerFaceColor',colorord(n,:))
            hold on
            plot(stateInfo.X(tim1:k,n)+20/2,stateInfo.Y(tim1:k,n)+50/2,'Color',colorord(n,:),'LineWidth',2)
        end

    end
    hold off
    pause(0.01)
end


% colorord=.25+.75*rand(size(XeT,2),3);
% 
%     writerObj = VideoWriter('AFL','MPEG-4');
%     writerObj.FrameRate = 10;
%     writerObj.Quality=100;
%     open(writerObj);
%     scrsz = get(0,'ScreenSize');
%     
%     for k=1:round(3*writerObj.FrameRate)
%         fig2=figure('Position',[scrsz(1) scrsz(2) scrsz(3) scrsz(4) ]);
%         set(fig2, 'Color',[0 0 0])
%         Mask=zeros(size(I),cl_image);
%         imshow(Mask)
%         text(v_image/2.2,u_image/2.5,'Detections','Color',[1 1 1],'FontSize',40)
%         
%         frame = getframe(fig2);
%         writeVideo(writerObj,frame);
%         close(fig2);
%     end
% 
%     
%  for k=1:Frame
%      fig2=figure('Position',[scrsz(1) scrsz(2) scrsz(3) scrsz(4) ]);
%      set(fig2, 'Color',[0 0 0])
%      filename = strcat([Image_address,'\'],file((k-1)*SRt+3).name);
%     I= imread(filename);
%     imshow(I), 
%     hold on
%     for jj=1:size(detections(k).xc,1)
%     rectangle('Position',[detections(k).xc(jj)-detections(k).w(jj)/2,...
%         detections(k).yc(jj)-detections(k).h(jj)/2,...
%         detections(k).w(jj),detections(k).h(jj)])
%     end
%     hold off
%     frame = getframe(fig2);
%     writeVideo(writerObj,frame);
%     close(fig2);
%  end
%  
%      for k=1:round(3*writerObj.FrameRate)
%         fig2=figure('Position',[scrsz(1) scrsz(2) scrsz(3) scrsz(4) ]);
%         set(fig2, 'Color',[0 0 0])
%         Mask=zeros(size(I),cl_image);
%         imshow(Mask)
%         text(v_image/2.2,u_image/2.5,'Tracking','Color',[1 1 1],'FontSize',40)
%         
%         frame = getframe(fig2);
%         writeVideo(writerObj,frame);
%         close(fig2);
%     end
%   
%  for k=1:Frame
%      fig2=figure('Position',[scrsz(1) scrsz(2) scrsz(3) scrsz(4) ]);
%      set(fig2, 'Color',[0 0 0])
%      filename = strcat([Image_address,'\'],file((k-1)*SRt+3).name);
%     I= imread(filename);
%     imshow(I),
%     hold on
%     for n=1:size(XeT,2)
%      ixt=sort(find(stateInfo.X(:,n)~=0));
%         tim1=max(k-l_t,ixt(1));
%         if (stateInfo.X(k,n))
%             rectangle('Position',[stateInfo.X(k,n)-20/2,stateInfo.Y(k,n)-50/2,20,50],'EdgeColor',colorord(n,:),'LineWidth',1)
%             
%             % rectangle('Position',[stateInfo.X(k,n)-stateInfo.W(k,n)/2,stateInfo.Y(k,n)-stateInfo.H(k,n),stateInfo.W(k,n),stateInfo.H(k,n)],'EdgeColor',colorord(n,:))
%             text(stateInfo.X(k,n),stateInfo.Y(k,n),num2str(n),'Color',colorord(n,:))
%             %          plot(XeT{1,n}(1,idx),XeT{1,n}(3,idx),'o','Color',colorord(n,:),'MarkerFaceColor',colorord(n,:))
%             hold on
%             plot(stateInfo.X(tim1:k,n)+20/2,stateInfo.Y(tim1:k,n)+50/2,'Color',colorord(n,:),'LineWidth',2)
%         end
%     end
%     hold off
%     frame = getframe(fig2);
%      writeVideo(writerObj,frame);    
%      close(fig2);
%  end
% 
% close(writerObj);
%    
% 
