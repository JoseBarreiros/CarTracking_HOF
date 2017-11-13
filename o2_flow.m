%% Detection of a moving object from two consequtive frames using Histogram of Flow: HOF
%  Optical Flow is computed using Horn-Schunck Method.
%  Intelligent Sensor Planning, Cornell University.
%  Jose Barreiros, 2017. PhD student.
%%
% Read in a video file.
vidReader = VideoReader('viptraffic.avi');

% Create optical flow object.
opticFlow = opticalFlowHS;

% Initialize variables
i_frame=51; %Frame that will be extracted and analysed.
h=3; %vertical grid for subplot
v=4; %horizontal grid for subplot
i=0;
%%
%%Read the video, extract the i_frame (i-1)_frame and compute Optical Flow.  
while hasFrame(vidReader)
    i=i+1;

    frameRGB = readFrame(vidReader);
    frameGray = rgb2gray(frameRGB);
     
    flow_temp = estimateFlow(opticFlow,frameGray); 
    if i==i_frame-1
    frameRGB_1=frameRGB ;
    
    end
      if i==i_frame
    frameRGB_2=frameRGB ;
    frameGray_p = rgb2gray(frameRGB_2);
    flow=flow_temp;
    end  

end
%%
hold off;

%Plot the (i-1)_frame
subplot(h,v,1);
imshow(frameRGB_1);
title('Previous frame');
hold on;

%Plot RGB image of i_frame.
subplot(h,v,2);
imshow(frameRGB_2);
title('Current frame Gray');

%Plot the i_frame and their correspondent optical flow in the same image.
subplot(h,v,3);
imshow(frameGray_p);
title('Current frame + Optical Flow');
hold on;
subplot(h,v,3);
plot(flow,'DecimationFactor',[5 5],'ScaleFactor',25);
q = findobj(gca,'type','Quiver');  % Find quiver handle
q.Color = 'r'; % Change color to red
q.LineWidth=1.5;

%Plot the Optical Flow.
subplot(h,v,4);
[x,y] = meshgrid(1:160,1:120);
quiver(x,-y,flow.Vx,flow.Vy);
title('Optical Flow');
axis([0 160 -120 0]);

%HOF.
subplot(h,v,5);
N=10; %number of bins
o=abs(flow.Magnitude); %Get Magnitude of the Optical Flow
histogram(o(o>(max(o(:))/N)),N);  %Compute and show the histogram of the
%magnitude of flow. The values from 0 to max(flow)/N have been cutted to improve visibility. 
title('HOF');

%Filter the image based on a threshold on HOF using a mask in matrix "o".
Thershold=0.02;
o(o<Thershold)=0; %All values below Threshold are set to 0
 o(o~=0)=1; %All values above Threshold are set to 1
 subplot(h,v,6);
 quiver(x,-y,flow.Vx.*o,flow.Vy.*o);  %Plot the masked/filtered optical flow
 title('Masked Optical Flow');
 axis([0 160 -120 0]);
 subplot(h,v,7);
 maskedBW=frameGray_p.*uint8(o); %Filter the image
 imshow(maskedBW);   %Plot the masked/filtered image
  title('Filtered Image');
 
%Filter the "salt and pepper" noise with a Median Filter
 subplot(h,v,8);
 p=medfilt2(maskedBW);
 imshow(p);
 title('Median Filter');

%Perform Morphological Close to make connected regions using connected
%components
 p(p>0)=255; %Threshold the image 
 se = strel('disk',6); %define the kernel to morphological closing
 closeBW = imclose(p,se);
  subplot(h,v,9);
 imshow(closeBW);
 title('Thresh. + Morph. Closing');
 CC = bwconncomp(closeBW);

%Apply the masked image from last step to the Current Frame to segment the 
%vehicle from the rest of the image.  
SegGray=frameGray_p.*(closeBW./255);
 subplot(h,v,10);
imshow(SegGray);
title('Segmented Gray');

%extract the Centroid of the Connected Regions
S = regionprops(CC,'Centroid');

%extract the Number of Pixels of the Connected Regions
area = cellfun(@numel,CC.PixelIdxList);

%Annotate the image with a Red Marker to show the centroid of the target, a
%Yellow Square to box the target and a Yellow Rectangle which includes the
%distance referenced to the camera FOV.
 for i=1:size(S,1)
   distance=sqrt((S(i).Centroid(1))^2+(S(i).Centroid(2))^2)  %Calculate the distance from centroid 
   %to the origin(0,0)-left corner of the image 
   marker_position=S(i).Centroid() ;
   ann_BW=insertMarker(closeBW,marker_position,'color','red','size',6);
   ann_Color=insertMarker(frameRGB_2,marker_position,'color','red','size',6);
   
   l=sqrt(area(i));
   ann_BW2=insertShape(ann_BW,'rectangle',[marker_position(1)-l/2 marker_position(2)-l/2 l l],'LineWidth',2);
   ann_Color2=insertShape(ann_Color,'rectangle',[marker_position(1)-l/2 marker_position(2)-l/2 l l],'LineWidth',2);
   
   text=strcat('x=',num2str(marker_position(1),3),' y=',num2str(marker_position(2),3),sprintf('\nd='),num2str(distance,4));
   d=2;
   ann_BW3=insertText(ann_BW2,[marker_position(1) marker_position(2)+l/2+d],text,'FontSize',8,'AnchorPoint','CenterTop');
   ann_Color3=insertText(ann_Color2,[marker_position(1) marker_position(2)+l/2+d],text,'FontSize',8,'AnchorPoint','CenterTop');
   
   closeBW=ann_BW3;
   frameRGB_2=ann_Color3;
   
 end

%Plot the annotated binary image 
subplot(h,v,11);
imshow(closeBW)
title('Annotated Image BW');
  
%Plot the annotated RGB image 
subplot(h,v,12);
imshow(frameRGB_2)
title('Annotated Image RGB');