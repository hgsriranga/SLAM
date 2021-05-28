close all
clear all

%% Small Run
graphSLAM= rosbag('PoseExtractor_gmapping.bag');
gmapping = rosbag('PoseExtractor_graphSLAM.bag');

graphPose = timeseries(graphSLAM);
gmappingPose = timeseries(gmapping);

xOffset= 0.51;
yOffset = -0.592;

graphPoseDataX = (graphPose.Data(:,4)+xOffset);
graphPoseDataY = (graphPose.Data(:,5)+yOffset);

gmapPoseDataX = (gmappingPose.Data(:,4)+xOffset);
gmapPoseDataY = (gmappingPose.Data(:,5)+yOffset);

% groundTruthX = [0 0 1.3 1.65 2.62 2.62 0];
% groundTruthY = [0 4.24 4.2 3.09 2.34 0 0];

groundTruthX = [zeros(1,102) linspace(0,1.3,23) linspace(1.3,1.65,27) linspace(1.65,2.62,23) 2.62*ones(1,57) linspace(2.62,0,41) zeros(1,16)];
groundTruthY = [zeros(1,20) linspace(0,4.24,68) linspace(4.24,4.2,47) linspace(4.2,3.09,14) linspace(3.09,2.34,30) linspace(2.34,0,38) zeros(1,72)];

a = 7;
H =[cosd(a), sind(a) ; -sind(a), cosd(a)];
gmapRot = H * horzcat(gmapPoseDataX,gmapPoseDataY)';
graphRot = H * horzcat(graphPoseDataX,graphPoseDataY)';

figure(1)
plot(groundTruthX,groundTruthY,'k','linewidth',2);
hold on
plot(gmapRot(1,:),gmapRot(2,:),'linewidth',1.5);
hold on
plot(graphRot(1,:),graphRot(2,:),'linewidth',1.5);



xlabel('X (m)','FontSize',14);
ylabel('Y (m)','FontSize',14);
legend('Ground Truth','GraphSLAM','Gmapping','FontSize',14);

% Distance Error

% gMapError = sqrt((gmapPoseDataX-groundTruthX').^2+(gmapPoseDataY-groundTruthY').^2);
% graphError = sqrt((graphPoseDataX-groundTruthX').^2+(graphPoseDataY-groundTruthY').^2);

dist = @(p1,p2) sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2);
GroundTruth = vertcat(groundTruthX,groundTruthY);
fastSLAM = horzcat(gmapPoseDataX,gmapPoseDataY)';
graphSLAM = horzcat(graphPoseDataX,graphPoseDataY)';

for i = 1:size(GroundTruth,2)
    gMapError(i) = dist(GroundTruth(:,i),gmapRot(:,i));
    graphError(i) = dist(GroundTruth(:,i),graphRot(:,i));
end

figure(2)
plot(gMapError,'linewidth',1.2);
hold on
plot(graphError,'linewidth',1.2);
xlabel('Time (samples)','FontSize',14);
ylabel('Distance Error (m)','FontSize',14);
legend('FastSLAM Error','GraphSLAM Error','Location','NorthWest','FontSize',14);


gmapRMSError = immse(vertcat(groundTruthX,groundTruthY),horzcat(gmapPoseDataX,gmapPoseDataY)');
graphRMSError = immse(vertcat(groundTruthX,groundTruthY),horzcat(graphPoseDataX,graphPoseDataY)');
%% Big Run
figure(3)

gmappingBig = rosbag('BigRun_Pose_Gmapping.bag');
gmappingPoseBig = timeseries(gmappingBig);

xOffset= 0.51;
yOffset = -0.592;

gmapPoseDataBigX = (gmappingPoseBig.Data(:,4)+xOffset);
gmapPoseDataBigY = (gmappingPoseBig.Data(:,5)+yOffset);

plot(gmapPoseDataBigX,gmapPoseDataBigY,'linewidth',1.5);
xlabel('X(m)')
ylabel('Y(m)')