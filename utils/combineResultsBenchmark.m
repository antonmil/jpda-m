function mr=combineResultsBenchmark(jobname,jobid,maxexper)

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

% concat gtInfo
allFgt=zeros(1,length(allscen));

fprintf('Concatenate ground truth\n');
[gtInfoAll,gtInfoSingle]=concatGT(allscenes,eval3d);


%% for all results
allmets=[];

dfile=sprintf('%s/res_%03d.txt',resdir,jobid);
if exist(dfile,'file'), delete(dfile); end
diary(dfile);

for r=1:maxexper
    fprintf('Experiment: %d\n',r);
    resfile=sprintf('%s/res_%03d.mat',resdir,r);
    if exist(resfile,'file')
        load(resfile);

        fprintf('*** 2D (Bounding Box overlap) ***\n');

        mets = fastCLEAR(mets2d, numel(find(gtInfoAll.Xi)),size(gtInfoAll.Xi,2),size(gtInfoAll.Xi,1));
        printMetrics(mets); fprintf('\n');
        allmets(r,:)=mets;
    end
end
diary off;

%%
% allmets=mets;


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


