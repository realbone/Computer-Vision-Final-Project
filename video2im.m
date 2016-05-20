clear all; 
vid=VideoReader('video7.mov');
 numFrames = vid.NumberOfFrames;
 n=numFrames;
 for i = 25:floor(n/2)-50
 frames = read(vid,2*i);
 
 c=size(frames,2);
 
 imwrite(frames(:,1:c/2,:),['video_7/left/image' int2str(i), '.jpg']);
 
 imwrite(frames(:,c/2+1:end,:),['video_7/right/image' int2str(i), '.jpg']);
 
 im(i)=image(frames);
 end