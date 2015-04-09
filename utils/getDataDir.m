function dataDir=getDataDir()
% set data directory

dataDir='/export/data1/data/';
if exist('/home/amilan','dir')
	dataDir='/home/amilan/research/projects/bmtt-data/';
end
if exist('d:/','dir')
    dataDir=fullfile('d:','research','projects','bmtt-dev','data',filesep);
end
