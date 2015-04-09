function allseq = parseSequences(sequence, dataDir)
% argument parsing -- sequence (e.g. TUD-Stadtmitte, AVG-TownCentre,...)
% returns a cell array with all the sequence names


% loop through all of them
if isempty(sequence)
	fprintf('loop through all sequences\n');
    dirItems=dir(dataDir);
    
    scnt=0;
    for d=1:length(dirItems)        
        if dirItems(d).isdir && ~strcmp(dirItems(d).name,'.') && ~strcmp(dirItems(d).name,'..') && ~strcmp(dirItems(d).name,'.hg') 
            scnt=scnt+1;
            allseq{scnt}=dirItems(d).name;
        end
    end
elseif ischar(sequence)
    allseq{1} = sequence;
elseif iscell(sequence)
    allseq = sequence;
elseif isnumeric(sequence) % 2el array, [train/test challenge]
    assert(length(sequence)==2,'indicator must be a two element array');
    assert(sequence(1)>=0 && sequence(1)<=1, 'train/test value must 0 or 1');
    assert(sequence(2)>=1 && sequence(2)<=4, 'challenge must be between 1 and 4');
    testTrainString='train'; if sequence(1)==1, testTrainString='test'; end
    chlString=['c' num2str(sequence(2))];
    seqmapFile=[chlString,'-',testTrainString,'.txt'];
    seqmapFile=[dataDir,seqmapFile]
    exist(seqmapFile,'file')
    assert(exist(seqmapFile,'file')>0,'seqmap file %s does not exist',seqmapFile);
    fid = fopen(seqmapFile);
    allseq = textscan(fid,'%s','HeaderLines',1);
    fclose(fid);
    allseq=allseq{1}';
%     allseq
    
else
    error('wrong argument');
end
