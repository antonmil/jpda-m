function fr = getFrameRate(sequence)

frameRateFile=[getDataDir,sequence,filesep,'framerate.txt'];
try
  fr = dlmread(frameRateFile);
catch err
  fr = 25;
  fprintf('Could not determine frame rate. Setting to 25 FPS!\n');
end