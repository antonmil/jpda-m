function trainModel(jobname,jobid,maxexper)

%% determine paths for config, logs, etc...
% addpath(genpath('./motutils/'));
format compact
addpath(genpath('utils'))
addpath(genpath('external'))

settingsDir=strsplit(jobname,'-');
runname=char(settingsDir{1})
learniter=char(settingsDir{2})
jid=char(settingsDir{3}); % unused

settingsDir=[runname '-' learniter];

confdir=sprintf('config/%s',settingsDir);

jobid=str2double(jobid);
confdir

resdir=sprintf('results/%s',settingsDir);
if ~exist(resdir,'dir'), mkdir(resdir); end
resdir


resultsfile=sprintf('%s/res_%03d.mat',resdir,jobid);

% define searchspace
searchspace={
    'Prun_Thre', ...
    'tret', ...
    'Term_Frame', ...
    'PD', ...
    'q1', ...
    'Mcov', ...
    'Upos', ...
    'Uvel', ...
    };

%%
% if computed alread, just load it
if exist(resultsfile,'file')
  load(resultsfile);
else

  conffile=fullfile(confdir,sprintf('%04d.ini',jobid));
  conffile

  inifile=fullfile(confdir,'0001.ini');
  inifile
  if ~exist(inifile,'file')
	  error('You must provide initial options file 0001.ini');
  end

  opt=parseOptions(inifile);

  %% zip up logs (previous)
  if jobid==1
    prevSetting=sprintf('%s-%d',runname,str2double(learniter)-1)
    zipstr=sprintf('!sh ziplogs.sh %s',prevSetting)
    eval(zipstr);
  end


  % take care of parameters
  jobid

  rng(jobid);
  % if jobid==1, leave as is
  if jobid==1
  % otherwise, randomize and write out new ini file
  else
	  ini=IniConfig();
	  ini.ReadFile(inifile);

	  params=[];
	  % we are only interested in [Parameters]
	  sec='Parameters';
	  keys = ini.GetKeys(sec);

	  for k=1:length(keys)
	      key=char(keys{k});
	      %params = setfield(params,key,ini.GetValues(sec,key));
          
            % if search parameter not in parameter list at all, ignore
            if ~ismember(key,fieldnames(opt.Parameters))
                continue;
            end
            if ~ismember(key,searchspace)
                continue; 
            end
            
	      params = [params ini.GetValues(sec,key)];
	  end

	  rnmeans = params; % mean values are the starting point
	  rmvars = rnmeans ./ 10; % variance is one tenth

	  if jobid <= maxexper/2
		  params = 2*rand(1,length(params)) .* rnmeans; % uniform [0, 2*max]
	  else
		  params = abs(rnmeans + rmvars .* randn(1,length(rnmeans))); % normal sampling
	  end


	  for k=1:length(keys)
	      key=char(keys{k});
            if ~ismember(key,searchspace)
                continue; 
            end
            
            % fix range
            if strcmp(key,'PD')
                params(k)=max(params(k),0.5);params(k)=min(params(k),0.99);
            elseif strcmp(key,'tret') || strcmp(key,'Term_Frame')
                params(k)=round(params(k));
            end
            
            
	      %params = setfield(params,key,ini.GetValues(sec,key));
	      opt.Parameters = setfield(opt.Parameters, key, params(k));
	  end

	  % write out new opt file
% 	  status = writeSTOptions(opt,conffile);
      writeOptions(opt.Parameters,conffile);
	  

	  
  end
  rng(1);

  allscensfile=fullfile(confdir,'doscens.txt');
  if ~exist(allscensfile,'file')
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
    
    
  else
    fid = fopen(allscensfile);
    allscenes=textscan(fid,'%s','HeaderLines',1);
    fclose(fid);
    allscenes=allscenes{1}';
  end
  allscenes
  allscen=1:11;

  learniter=str2double(learniter)

  mets2d=zeros(max(allscen),14);
  mets3d=zeros(max(allscen),14);
  ens=zeros(max(allscen),1);

  scenario=0;
  for scen=allscenes
      scenario=scenario+1;
	  fprintf('jobid: %d,   learn iteration %d\n',jobid,learniter);
	  scensolfile=sprintf('%s/prt_res_%03d-scen%02d.mat',resdir,jobid,scenario)

	  try
	    load(scensolfile);
	  catch err
	    fprintf('Could not load result: %s\n',err.message);
	    try 
	      [metrics2d, metrics3d, stateInfo]=runMFJPDA(char(scen),conffile);
	    catch err2
	    fprintf('Tracking failed: %s\n',err2.message);
        error('failure');
	      metrics2d=zeros(1,14);
	      metrics3d=zeros(1,14);
	      
	      [seqName, seqFolder, imgFolder, imgExt, seqLength, dirImages] = ...
		  getSeqInfo(scen, getDataDir);
	      
	      stateInfo.X = zeros(seqLength, 0);
	      stateInfo.Y = zeros(seqLength, 0);
	      stateInfo.Xi = zeros(seqLength, 0);
	      stateInfo.Yi = zeros(seqLength, 0);
	      stateInfo.W = zeros(seqLength, 0);
	      stateInfo.H = zeros(seqLength, 0);
	      stateInfo.frameNums = 1:seqLength;
	      stateInfo.opt.track3d=0;stateInfo.opt.cutToTA=0;
	      stateInfo.sceneInfo = [];
	      
    
	    end
	    save(scensolfile,'stateInfo','metrics2d','metrics3d');
	  end	  

	  mets2d(scenario,:)=metrics2d;
	  mets3d(scenario,:)=metrics3d;
	  infos(scenario).stateInfo=stateInfo;

  end


    
  save(resultsfile,'opt','mets2d','mets3d','ens','infos','allscen');
  
  
  % remove temp scene files
  for scenario=allscen
    scensolfile=sprintf('%s/prt_res_%03d-scen%02d.mat',resdir,jobid,scenario)
    if exist(scensolfile,'file')
      delete(scensolfile);
    end
  end
end

%%
% evaluate what we have so far
% bestexper=combineResultsRemote(settingsDir);
bestexper=combineResultsBenchmark(settingsDir,jobid,maxexper);

resfiles=dir(sprintf('%s/res_*.txt',resdir))
fprintf('done %d experiments\n',length(resfiles));

% querystring=sprintf('qstat -t | grep %s | wc -l',settingsDir)
% [rs,rjobs] = system(querystring) 
% rjobs=str2double(rjobs)-1; % subtract currently running
% 
% 
% fprintf('%d other jobs still running\n',rjobs);

rjobs = maxexper-length(resfiles);
fprintf('%d other jobs still running\n',rjobs);


% if last one, resubmit
if bestexper==1 && length(resfiles)==maxexper
	fprintf('Training Done!');
else
%   if rjobs<=0
  if rjobs==0
	fprintf('resubmitting ... \n');
    runname
    if ischar(learniter), learniter=str2double(learniter); end
	newSetting=sprintf('%s-%d',runname,learniter+1)	
	newConfdir=sprintf('config/%s',newSetting)
	cpstr=sprintf('!cp -R %s %s',confdir,newConfdir)
	fprintf('copy config dir\n');
	eval(cpstr);
	
	% copy relevant config into first one
	conffile=sprintf('%s/%04d.ini',confdir,bestexper)
	cpstr=sprintf('!cp %s %s/0001.ini',conffile,newConfdir)
	fprintf('copy best config file');
	eval(cpstr);
	
	% 
	submitstr=sprintf('!ssh moby \"cd research/projects/jpda-m; sh submitTrain.sh %s\"',newSetting)
	fprintf('submit: %s\n',newSetting)
  	eval(submitstr);	
  else
    fprintf('waiting for other jobs to finish\n');
  end
end

