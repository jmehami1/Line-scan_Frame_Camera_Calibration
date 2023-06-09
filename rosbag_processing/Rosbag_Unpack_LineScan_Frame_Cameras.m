% Used to unpack a Rosbag that contains the following:
% - line-scan frame camera calibration images
%
% The rosbag is provided by the user, and the extracted images are saved
% into a directory with the same name as the Rosbag in the same location as
% the Rosbag
%
% The closest corresponding images in time are acquired and stored.
%
% Author: Jasprabhjit Mehami, 13446277

clc;
close all;
clear;

%% Get rosbag details from user

%rosbag name and its path
[rosbagName, rosbagPath] = uigetfile(['~', filesep, '*.bag'], 'Please provide location of recorded calibration ROSBAG');

disp("Checking Rosbag...")

%full path to rosbag
rosbagPathFull = fullfile(rosbagPath, rosbagName);
%open rosbag
bag = rosbag(rosbagPathFull);

%remove the file-type .bag from name
rosbagName = rosbagName(1:end-4);

% Directory where images will be extracted to.
mainImgDir = fullfile(rosbagPath, rosbagName);

%get topic names
topics = bag.AvailableTopics.Row;

%check if bag has correct number of topics
if  length(topics) < 2
    error("There should be atleast two topics stored in the bag");
end

%extract topic names
for i = 1:length(topics)
    if strcmp(topics{i}, "/dark_ref")
        continue;
    elseif strcmp(topics{i}, "/Resonon/image_raw") || strcmp(topics{i}, "/Goldeye/image_raw")
        lsTopic = topics{i};
    else
        frameTopic = topics{i};
    end
end

if ~exist('frameTopic',"var")
    error("Missing frame camera topic in rosbag");
end

if ~exist('lsTopic',"var")
    error("Missing line-scan hyperspectral camera topic in rosbag");
end

%% ROSBAG extraction parameters

disp("Setting extraction parameters...")

prompt = {'Image skip rate (extract every n-th image from bag):','Maximum allowed time difference between camera frames in milliseconds:'};
dlgtitle = 'Input';
dims = [1 35];
definput = {'5','20'};
answer = inputdlg(prompt,dlgtitle,dims,definput);

if isempty(answer)
    answer = definput;
end

IMG_SKIP = str2double(answer{1});
MAX_TIME_DIF = str2double(answer{2})/1000; %units are in seconds

fprintf('\tSkipping every %i-th image\n', IMG_SKIP);
fprintf('\tMaximum time difference between corresponding frames is %.3g seconds\n', MAX_TIME_DIF);

%% Create directories

disp("Creating directories...")

%check if main directory exist, if so, delete it
if exist(mainImgDir, 'dir')
    prompt =  [rosbagName, ' already exists, would you like to delete it?'];
    dlgtitle = 'Delete existing folder';
    definput = {'5','20'};
    answer = questdlg(prompt,dlgtitle,'Cancel');

    if strcmpi(answer, "yes")
        rmdir(mainImgDir, 's');
    else
        error("Extracted folder already exists. Nothing done.")
    end
end

mkdir(mainImgDir);

frameDir = fullfile(mainImgDir, 'Frame');
mkdir(frameDir);

lsDir = fullfile(mainImgDir, 'Line-scan');
mkdir(lsDir);

%% unpack the rosbag and find corresponding images

disp('Extracting images from rosbag...')
[lsImages, frameImages, numImages] = RosbagUnpackCorresImages(bag, lsTopic, frameTopic, MAX_TIME_DIF, IMG_SKIP);

%% Save images to directory

disp('Saving images to directory ...');

for i = 1:numImages
    curLS = lsImages{i};
    hsFileName = fullfile(lsDir, ['hs',num2str(i),'.png']);

    curFrame = frameImages{i};
    frameFileName = fullfile(frameDir, ['img',num2str(i), '.png']);

    imwrite(curLS, hsFileName);
    imwrite(curFrame, frameFileName);
end

disp('Completed!!');

