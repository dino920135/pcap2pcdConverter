% This code convert the .pcap file from Velodyne Lidar to .pcd file for
% each frame with velodyneFileReader().
% 
% Project support: MOI
% Author: Tsai Syun
% Update time: 2020.11.13
% Other functions: /lib
%   calcN.ma
%   GpsConstants.m
%   JulianDay.m
%   LeapSeconds.m
%   plh2xyz.m
%   Utc2Gps.m
% 
% The issue of initial time jump across hours between the file name and
% .pcap is fixed with checking start time

clearvars -except dirName
clc
if ~exist('dirName','var')
    dirName = '';
elseif dirName == 0
    dirName='';
end
addpath('lib')
%% SETTINGS
AutoGetStartTime = 1;   % auto get start time from .pcap file name
frameMaxNum = 10;       % how many frames to transfer
savePath = 'PCD/';      % path to save .pcd file
UTF_timeZone = 8;       % time zone of the collected data
deviceModel = 'VLP16';

%% READ FILES
[prFileName,dirName] = uigetfile({...
    '*.pcap','PCAP Files(*.pcap)';'*.*','All Files(*.*)'},'Choose PCAP File',dirName);
if prFileName == 0, disp('No file is selected.'); return; end
fileName = [dirName,prFileName];

veloReader = velodyneFileReader(fileName,deviceModel);

if AutoGetStartTime == 1
    tmpCell = strsplit(prFileName, {'-','_'});
    utcStr = zeros(1,6);
    for i = 1:6
        utcStr(i) = str2num(tmpCell{i});
    end
    
%     check start time
    sec = mod(seconds(veloReader.CurrentTime),60);
    min = floor(mod(minutes(veloReader.CurrentTime),60));
    if min - utcStr(5) > 30
        utcStr(4) = utcStr(4) - 1;
    elseif min - utcStr(5) < -30
        utcStr(4) = utcStr(4) + 1;
    end
    utcStr(5:6) = [min, sec];
end

% Convert veloTime to GPST(time since 1980.01.06)
veloTime = [veloReader.StartTime, veloReader.StartTime:veloReader.Duration/veloReader.NumberOfFrames:veloReader.EndTime]';
veloGPST = [0; zeros(veloReader.NumberOfFrames,1)];
crossHr = 0;
for i = 2:veloReader.NumberOfFrames + 1
    sec = mod(seconds(veloTime(i,1)),60);
    min = floor(mod(minutes(veloTime(i)),60));

    utcStr(5:6) = [min,sec];
%     [GPST, ~] = Utc2Gps(utcStr);                  
%     veloGPST(i) = GPST(2) - UTF_timeZone*3600;    %GPS week second
    [~, GPST] = Utc2Gps(utcStr);    
    veloGPST(i) = GPST - UTF_timeZone*3600;  %GPS time since 1980.01.06
    
    % check time continue
    if abs(veloGPST(i) - veloGPST(i-1) + 3600) <= 10
        crossHr = floor((veloGPST(i-1) - veloGPST(i)) / 3599);
    end
    
    veloGPST(i) = veloGPST(i) + crossHr * 3600;        
end
veloTime(1) = [];
veloGPST(1) = [];


%% SHOW POINT CLOUDS & SAVE .PCD
xlimits = [-60 60];
ylimits = [-60 60];
zlimits = [-20 20];

player = pcplayer(xlimits,ylimits,zlimits);

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');

veloReader.CurrentTime = veloReader.StartTime;
frameCount = 0;
while(hasFrame(veloReader) && player.isOpen() && frameCount <= frameMaxNum )
    frameCount = frameCount + 1;
    ptCloudObj = readFrame(veloReader);
    view(player,ptCloudObj.Location,ptCloudObj.Intensity);
        
    pcwrite(ptCloudObj,[savePath num2str(veloGPST(frameCount)) '.pcd'],'Encoding','ascii');
    pause(0.1);
end