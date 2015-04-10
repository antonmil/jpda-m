function rt=getRuntimes(sequence)
%%

allseq=parseSequences(sequence,getDataDir);

rt=0;

scnt=0;
for seq=allseq
    scnt=scnt+1;
    srt = load(sprintf('/home/amilan/research/projects/bmtt-dev/results/MFJPDA/runtimes/%s.txt',char(seq)));
    rt(scnt)=srt;
end

