% points_set={};
% size_set=[];
% for i=25:300
%     im=imread(['video_7/left/image',num2str(i),'.jpg']);
%     [imPoints,size]=detectCheckerboardPoints(im);
%     points_set{i}=imPoints;
%     size_set(i,:)=size;
%     imshow(im);hold on;plot(imPoints(:,1),imPoints(:,2),'go');pause(0.2);hold off;
% end


clear all;
imageFileNames={};
  for i = 25:300
      imageFileName = ['video_7/right/image',num2str(i), '.jpg', ];
      imageFileNames{i-24} =imageFileName;
  end
  [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);

  imageFileNames = imageFileNames(imagesUsed);
  for i = 1:numel(imageFileNames)
      I = imread(imageFileNames{i});

      imshow(I); hold on; plot(imagePoints(:,1,i), imagePoints(:,2,i), 'ro');pause(0.01);hold off;
  end