%Camera calibration of a linescan and Frame camera system in a probablistic
%manner using multiple views of a planar pattern.
%This is the main script to run the calibration framework in an active
%manner where images that only add value to the calibration are kept.
%This is for the REAL camera system

clc;
close all;
clear;

%path to all calibration code functions
addpath('src');

%external library directory
addpath('ext_lib');

%check if mex_ChArUco_Pose has been built
if ~exist(fullfile("ext_lib", "mex_ChArUco_Pose", "bin", "ArucoPixDect.mexa64"), 'file')
    error("Please build mex_ChArUco_Pose submodule")
else
    addpath(genpath(fullfile("ext_lib", "mex_ChArUco_Pose")));
end

%robotics toolbox
run(fullfile('ext_lib', 'rvctools', 'startup_rvc.m'));

%yaml reader package
addpath(genpath(fullfile('ext_lib', 'yamlmatlab-master')));

%parameter file
paramFile = fullfile('parameter_files', 'calibration.yaml');
if ~exist(paramFile, 'file')
    error("YAML parameter file not found");
end

%pattern specifications
patternFile = fullfile('parameter_files', 'pattern.yaml');
if ~exist(patternFile, 'file')
    error("YAML file containing pattern parameters not found");
end


%% Load parameters

disp("Loading calibration parameters...");

paramYaml = yaml.ReadYaml(paramFile);
displayOn = paramYaml.display_on;
frameCamera = paramYaml.frame_camera_name;
steadstate_readings = paramYaml.steadystate_readings; %number of readings until active calibration algorithm has considered to reached a steadystate in the calibration
min_eigen_value = paramYaml.minimum_eigen_value; %minimum magnitude to eigenvalues (avoids having to deal with 0)
eigRange = cell2mat(paramYaml.eig_values_considered); %the eigenvalues which will only be considered by the algorithm
SUM_THRES = paramYaml.sum_eigen_threshold; %the thresold for the summed normalised metric (should be zero to ensure the sum is greater than zero, which relates in an increase in information with the current images.)

%if the line-scan camera is mounted upside down, then this needs to be set to true.
flipLS_img = paramYaml.flip_linescan_img;
naiveCalibration = paramYaml.naive_calibration;

%Approximate translation in x and z direction to ensure optimisation does
%not reduce to mirror pose
t3Act = paramYaml.t3_approximate;
t1Act = paramYaml.t1_approximate;

%algorithm of choice for solving the optimisation
optmAlgStruct.algo = paramYaml.algorithm;

%skip covariance only if not performing active calibration
optmAlgStruct.skipCov = paramYaml.skip_covariance && naiveCalibration;
stdImgCoor = paramYaml.std_pixel_error; %variance in the pixel coordinates for both cameras

% Upper and lower bounds for trust-region-reflective algorithm
lowerBounds = cell2mat(paramYaml.lower_bounds);
upperBounds = cell2mat(paramYaml.upper_bounds);


%Read YAML file containing the pattern specifications
% All dimensions are in metres
pattern = yaml.ReadYaml(patternFile);
numLines = pattern.numLines;

%% Check files and data directories are correct

disp("Checking files and directories...");


%frame camera intrinsic parameters file
frameIntrFile = fullfile('frame_camera_intrinsic', [frameCamera, '.mat']);
if ~exist(frameIntrFile, 'file')
    error("Frame camera intrinsic parameters not found for %s", frameCamera);
end

%Get source directory where images are located and results will be saved
srcDir = uigetdir(['~', filesep], 'Provide source directory where images are located?');

lsDir = fullfile(srcDir, 'Line-scan');
frameDir = fullfile(srcDir, 'Frame');

if ~exist(lsDir, 'dir')
    error("could not find line-scan hyperspectral directory");
end

if ~exist(frameDir, 'dir')
    error("could not find Frame directory");
end

fullPathLS = fullfile(lsDir, '*.png');
fullPathFrame = fullfile(frameDir, '*.png');

%Need to get number of images in directory
numImagesLS = numel(dir(fullPathLS));
numImagesFrame = numel(dir(fullPathFrame));

if numImagesLS ~= numImagesFrame
    error("number of images in directories are not the same");
end

numImages = numImagesLS;

% Results directory
resultDir = fullfile(srcDir, 'calibration_results');
%create results directory if not made
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

%% Load intrinsic parameters

disp("Loading frame camera intrinsic parameters...");

load(frameIntrFile); %Load the intrinsic parameters of the camera

%extract focal point components in pixels
fx = cameraParams.FocalLength(1);
fy = cameraParams.FocalLength(2);

%optical centre of camera in pixelstrue
u0 = cameraParams.PrincipalPoint(1);
v0 = cameraParams.PrincipalPoint(2);

%array of intrinsic parameters that introduce noise (assume noise in
%intrinsic distortion is zero)
thetaFrameintr = [fx, fy, u0, v0];

%distortion parameters
distRad = cameraParams.RadialDistortion;
distTan = cameraParams.TangentialDistortion;
distCoefCV = [distRad(1:2), distTan, distRad(3)]; %array of distortion coefficients in opencv format

if ~optmAlgStruct.skipCov
    if exist('estimationErrors', 'var')
        %extract the error in the intrinsic parameters of the frame camera
        std_f = estimationErrors.IntrinsicsErrors.FocalLengthError;
        std_u0v0 = estimationErrors.IntrinsicsErrors.PrincipalPointError;
        stdFrameIntr = [std_f,std_u0v0];
    else
        %assume no error in frame camera intrinsic
        stdFrameIntr = zeros(1,4);
    end
end

%% Load images for both cameras

disp("Loading undistorted frame and line-scan images...");

%Preallocate space for cell arrays of images
imagesLS = cell(1,numImages);
imagesFrame = cell(1,numImages);

% Load all images
for i = 1:numImages
    imagesFrame{i} = undistortImage(imread(fullfile(frameDir, ['img', num2str(i),'.png'])), cameraParams);
    imagesLS{i} = imread(fullfile(lsDir, ['hs', num2str(i),'.png']));
end

%Size of the images from the RGB camera
frameImgSize = size(imagesFrame{1});
frameImgSize = frameImgSize(1:2);

%% Get pose of the plane using the ArUco pattern
 
disp("Estimating pose of Aruco board...");

%ChArUco pattern size
xNumMarker = pattern.NumCols;
yNumMarker = pattern.NumRows;
arucoLen = pattern.ArucoSideLength;
sepLen = pattern.SeparationLength;
numMarkersExp = pattern.NumberExpectedMarker;

%intrinsic object for the RGB camera
frameIntrinsic = cameraIntrinsics(thetaFrameintr(1:2),thetaFrameintr(3:4), frameImgSize);
kFrame = frameIntrinsic.IntrinsicMatrix';

%store all the poses of each found pattern
extPosePattern = zeros(4,4,numImages);

%again, used to filter images where the pose can't be found
goodImages = zeros(1,numImages);
numGoodImg = 0;

if displayOn
    figImgExt = figure('Name','ArUco pattern pose');
end

% 2D marker corner positions in world coordinates (metres)
markerCornerCell = ArUcoBoardMarkerCornersCell(0, xNumMarker, yNumMarker, arucoLen, sepLen);

% Estimate extrinsic pose using ArUco board
for imgLoop = 1:numImages
    [rotMat, trans, found, imgDisp] = ArucoPosEst(imagesFrame{imgLoop}, markerCornerCell, cameraParams, false);

    if ~found
        continue;
    end

    %image is good
    numGoodImg = numGoodImg + 1;
    goodImages(numGoodImg) = imgLoop;

    %store found extrinsic parameter
    extPosePattern(:,:,numGoodImg) = [rotMat,trans'; 0, 0, 0, 1];

    %display the frame camera image with the projected axis on the pattern
    if displayOn
        clf(figImgExt);
        imshow(imgDisp); hold on;
        drawnow();
    end

end


%remove all data from the frame images where we could not find proper
%extrinsic parameters
goodImages = goodImages(1:numGoodImg);
imagesFrame = imagesFrame(goodImages);
imagesLS = imagesLS(goodImages);
extPosePattern = extPosePattern(:,:,1:numGoodImg);
numImages = numGoodImg;


%% Find all the linescan intersection points across all images in the linescan image coordinates

disp('Finding line-scan lines in images...')
lsPointsImgCoords = zeros(numLines,numImages);

%used to filter images that don't have correct number of edges
goodImages = zeros(1,numImages);
numGoodImg = 0;

if displayOn
    figLineScanImg = figure('Name', 'Linescan intersection points identified');
end

for i = 1:numImages
    %detect lines from line-scan image

    curImg = imagesLS{i};
    hsPts = DetectHS_LineScanLines(curImg, numLines);

    %could not find correct lines from the image, ignore this image
    if hsPts < 0
        continue;
    end

    %this image is good, found all lines
    numGoodImg = numGoodImg +1;
    goodImages(numGoodImg) = i;

    %if the camera is upside down, then the left side of the image is
    %actually the right side of the pattern. So we need to flip the array
    %of pixel locations
    if flipLS_img
        lsPointsImgCoords(:,numGoodImg) = hsPts(end:-1:1); %flip array of pixel locations
    else
        lsPointsImgCoords(:,numGoodImg) = hsPts;
    end

    %displays the hyperspectral line-scan image with red vertical lines
    if displayOn
        clf(figLineScanImg);

        [bands, pixels] = size(curImg);

        if bands < 256
            bandFactorIncrease = ceil(256/bands);
            curImg = repmat(curImg, [bandFactorIncrease,1,1]);
            bands = size(curImg,1);
        end

        imshow(curImg); hold on;

        for j = 1:length(hsPts)
            figure(figLineScanImg)
            line( [hsPts(j) hsPts(j)], [0 bands], 'Color', 'red');
            text( hsPts(j)+4, 100, num2str(j), 'Color', [0 1 0]);
        end
        drawnow;
    end
end

[~, numPixLS] = size(curImg);

%remove all the data from the images that we could not find linescan
%lines for.
goodImages = goodImages(1:numGoodImg);
lsPointsImgCoords = lsPointsImgCoords(:,1:numGoodImg);
extPosePattern = extPosePattern(:,:,goodImages);

imagesFrame = imagesFrame(goodImages);
imagesLS = imagesLS(goodImages);
numImages = numGoodImg;


%% Find the linescan intersection points in the pattern coordinate system

disp('Determining line-scan intersection points in pattern coordinate system...');

lsPointsPatCoords = cell(1,numImages);
lsPointsPatCoordsERR = cell(1,numImages);

%used to filter images where the extracted data points do not form a
%straight line (line straightness is not good)

goodImages = zeros(1,numImages);
numGoodImg = 0;

if displayOn
    figViewLinePat = figure('Name', 'view line on the pattern');
end

for i = 1:numImages
    [curlsPointsPatCoords, curlsPointsPatCoordsERR] = CrossRatioPatternPoints(pattern, lsPointsImgCoords(:,i), stdImgCoor^2);

    %extracted line-scan line is good
    numGoodImg = numGoodImg + 1;
    goodImages(numGoodImg) = i;


    lsPointsPatCoords{numGoodImg} = curlsPointsPatCoords;
    lsPointsPatCoordsERR{numGoodImg}  = curlsPointsPatCoordsERR;

    if displayOn
        clf(figViewLinePat);
        DisplayPattern(figViewLinePat, pattern, curlsPointsPatCoords, eye(3), true);
    end
end

%remove all indexes which had lines that were not straight
goodImages = goodImages(1:numGoodImg);
lsPointsImgCoords = lsPointsImgCoords(:,goodImages);
imagesFrame = imagesFrame(goodImages);
imagesLS = imagesLS(goodImages);
extPosePattern = extPosePattern(:,:,goodImages);
lsPointsPatCoords = lsPointsPatCoords(1:numGoodImg);
lsPointsPatCoordsERR = lsPointsPatCoordsERR(1:numGoodImg);
numImages = numGoodImg;

%% Transform linescan pattern points to the frame camera coordinate system

disp('Transforming line-scan intersection points to frame camera coordinate system...')


linePtsTransFrame =  zeros(numLines,3, numImages);

for i = 1:numImages
    curExt = extPosePattern(:,:,i);
    %Turn feature points on pattern into homogenous coordinates
    linePtsPattern = [[lsPointsPatCoords{i}]';ones(1,numLines)];

    %Transform the points from the pattern to the frame coordinate frame
    ptsTrans = (curExt*linePtsPattern)';

    linePtsTransFrame(:,:,i) = ptsTrans(:,1:3);
end

%calculate the uncertainty in the transformed linescan feature points in
%the frame camera
if ~optmAlgStruct.skipCov
    linePtsTransFrame_ERR = UncertaintyFrameCameraParam(pattern, lsPointsPatCoords, lsPointsPatCoordsERR, numImages, stdImgCoor, stdFrameIntr, thetaFrameintr, distCoefCV, frameImgSize, imagesFrame, extPosePattern);
else
    linePtsTransFrame_ERR = 0;
end

%plot all poses of the pattern with the line-scan lines. Plot the projected
%view-line of the pattern onto the corresponding frame image.
if displayOn
    patFig = figure('Name', 'Multiple patterns in Frame Camera Coordinate Frame');
    plotCamera('Location', zeros(1,3), 'Orientation', eye(3), 'Size', 0.05, 'AxesVisible', true); hold on;

    %plot all poses of the pattern and view-line seen by line-scan camera
    for i = 1:numImages
        %Transform the points from the pattern to the frame coordinate frame
        ptsTrans = linePtsTransFrame(:,:,i);
        %plot the position of the points on the pattern
        plot3(ptsTrans(:,1), ptsTrans(:,2), ptsTrans(:,3), '-x', 'LineWidth', 1.5, 'MarkerFaceColor', [0,1,0]); hold on;
    end

    xlabel('x');
    ylabel('y');
    zlabel('z');
    grid on;
    axis equal;
end



%% Project linescan points onto the frame camera image

if displayOn

    projectLineFig = figure('Name', 'Projected Line');

    for i = 1:numImages
        figure(projectLineFig);
        transPts = linePtsTransFrame(:,:,i);

        %project points to image frame
        lineImgPts = projectPoints(transPts, kFrame, eye(4), distCoefCV, frameImgSize);

        frameImg = imagesFrame{i};
        clf(projectLineFig);
        imshow(frameImg);hold on;

        %plot projected points onto image frame
        plot(lineImgPts(:,1), lineImgPts(:,2), 'ro-', 'LineWidth',2, 'MarkerFaceColor', [0.3010 0.7450 0.9330]);

        for j = 1:numLines
            text(lineImgPts(j,1)+5, lineImgPts(j,2)+5, int2str(j), 'Color','cyan');
        end
        hold off;
        drawnow();
    end
end

%% Cut down on the number of images used to calibrate

% rearrange data in random order
disp('Random ordering of images for calibration...')
rng(0);
orderImg = randperm(numImages);
linePtsTransFrame = linePtsTransFrame(:,:,orderImg);
lsPointsImgCoords = lsPointsImgCoords(:,orderImg);
imagesFrame = imagesFrame(orderImg);
imagesLS = imagesLS(orderImg);
extPosePattern = extPosePattern(:,:,orderImg);
lsPointsPatCoords = lsPointsPatCoords(orderImg);
lsPointsPatCoordsERR = lsPointsPatCoordsERR(orderImg);

%% Set optimisation options

%trust region algorithm
optmAlgStruct.optOptionsTrust = optimoptions('lsqnonlin', 'Algorithm', 'trust-region-reflective', 'SpecifyObjectiveGradient',true, 'CheckGradients', false, ...
    'MaxIterations', 10000, 'FunctionTolerance',1e-7, 'MaxFunctionEvaluations',1000000000, 'StepTolerance',1e-6, 'OptimalityTolerance', 1e-7);
optmAlgStruct.optOptionsTrust.Display = 'none';

%levenberg-marquardt
optmAlgStruct.optOptionsLeven = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', 'SpecifyObjectiveGradient',true, 'CheckGradients', false, ...
    'MaxIterations', 1000, 'FunctionTolerance',1e-5, 'MaxFunctionEvaluations',1000000000, 'StepTolerance',1e-6,'ScaleProblem', ...
    'jacobian', 'InitDamping', 1, 'FiniteDifferenceType', 'central', 'UseParallel', false);
optmAlgStruct.optOptionsLeven.Display = 'none';

optmAlgStruct.LBounds = lowerBounds;
optmAlgStruct.UBounds = upperBounds;

%% Perform calibration of line-scan and frame camera system

disp(' ');
disp('###Starting calibration###');

paramHeaders = ["tx (m)", "ty (m)", "tz (m)", "rz (rad)", "ry (rad)", "rx (rad)", "fy (pixels)", "v0 (pixels)", "K1", "K2", "T1"];

if naiveCalibration
    disp(['Naive calibration of all ', num2str(numImages), ' images']);

    numOpt = 1;

    %start timer
    tic();

    [linescan_Opt_Param, linescan_Opt_ParamErr, eigFIM, rankFIM] = ...
            MainLineScanCalibration(lsPointsImgCoords, linePtsTransFrame, ...
            linePtsTransFrame_ERR, stdImgCoor, numImages, pattern, t3Act, t1Act, optmAlgStruct);

    toc();
    disp('****Optimisation ending****');

else
    numThetaLS = 11; %number of unknown parameters for the linescan camera

    %These results will be acquired from the calibration and stored
    rankFIM = zeros(1,numImages-1);
    eigFIM = zeros(numThetaLS, numImages-1);
    linescan_Opt_Param = zeros(numImages - 1, numThetaLS);
    linescan_Opt_ParamErr = zeros(numThetaLS, numThetaLS, numImages - 1);
    sumEigRelChange = zeros(1,numImages-1); %sum of the total relative change in eigen values

    disp(['Active calibration of all ', num2str(numImages), ' images ...']);
    
    %start timer
    tic();

    numOpt = 0;

    %assume first image is good
    goodImages = 1;

    %Loop through all provided images of the calibration
    for imgLoop = 2:numImages
        numOpt = numOpt + 1; %new optimisation with an added new image

        %The set of images used for calibration
        curImages =  [goodImages, imgLoop];
        goodImages = curImages;

        %Perform the calibration with the set of images and acquire results
        [cur_linescan_Opt_Param, cur_linescan_Opt_ParamErr, cur_eigFIM, cur_rankFIM, curRes] = ...
            MainLineScanCalibration(lsPointsImgCoords(:,curImages), linePtsTransFrame(:,:,curImages), ...
            linePtsTransFrame_ERR(:,:,:,curImages), stdImgCoor, numOpt+1, pattern, t3Act, t1Act, optmAlgStruct);

        %ignore result if fy or v0 is negative
        if any(cur_linescan_Opt_Param([7,8]) < 0)
            goodImages(end) = [];
            numOpt = numOpt - 1;
            disp(['Image ', num2str(imgLoop), ': Negative fy or v0']);
            continue;
        end

        %Ignore any imaginary components of the eigenvalues. This can occur for
        %parameters where the information is insufficient for calibration
        %(typically the distortion parameters)
        cur_eigFIM = real(cur_eigFIM);

        %First two images always calibrated. Assume this to be baseline.
        %Any added images which result in lower information metric will be
        %rejected
        if numOpt < 2
            rankFIM(1) = cur_rankFIM;
            eigFIM(:,1) = cur_eigFIM;
            prev_eigFIM = cur_eigFIM;
            prevRes = curRes;

            linescan_Opt_Param(1, :) = cur_linescan_Opt_Param;
            linescan_Opt_ParamErr(:,:,1) = cur_linescan_Opt_ParamErr;

            sumEigRelChange(1) = 0;
            disp(['***************Image ', num2str(imgLoop), ' Added*************']);
            continue;
        end

        curSumDiff = 0;

        for i = eigRange
            if prev_eigFIM(i) > min_eigen_value
                %total sum of relative change in eigen values
                curSumDiff = curSumDiff + (cur_eigFIM(i) - prev_eigFIM(i))./prev_eigFIM(i);
            end
        end

        %This sum needs to be greater than threshold, otherwise the observability has gone down
        if (curSumDiff < SUM_THRES)
            goodImages(end) = [];
            numOpt = numOpt - 1;
            disp(['Image ', num2str(imgLoop), ': Eigen sum decreases']);
            continue;
        end

        prevRes = curRes;
        prev_eigFIM = cur_eigFIM;

        tableOptimisedParameters = array2table(cur_linescan_Opt_Param, 'VariableNames', paramHeaders);
        disp(tableOptimisedParameters);

        %store all the information for the current iteration with the current
        %set of images
        rankFIM(numOpt) = cur_rankFIM(end);
        eigFIM(:,numOpt) = cur_eigFIM(:,end);
        linescan_Opt_Param(numOpt, :) = cur_linescan_Opt_Param(end,:);
        linescan_Opt_ParamErr(:,:,numOpt) = cur_linescan_Opt_ParamErr(:,:,end);
        sumEigRelChange(numOpt) = curSumDiff;

        disp(['***************Image ', num2str(imgLoop), ' Added*************']);

        %Check if we have reached steadystate to prematurly stop the
        %calibration
        if numOpt > steadstate_readings
            %detect steady state of total sum of relative change in eigen values to
            %end calibration. Determine the derivative of the total sum of relative change in eigen
            %values
            difChange = abs(diff(sumEigRelChange(1:numOpt)));

            % Use sliding window to find windows of consecutive elements below
            % a difference of 1
            steady = difChange((end - steadstate_readings+1):end) < 1;

            %Check if steadystate has happened in the current set of images
            steadyReached = all(steady);

            if steadyReached
                toc();
                disp('****Optimisation ending****');
                break;
            end

        end
    end

    %clip the arrays/matrices upto the last optimisation
    rankFIM = rankFIM(1:numOpt);
    eigFIM = eigFIM(:,1:numOpt);
    linescan_Opt_Param = linescan_Opt_Param(1:numOpt, :);
    linescan_Opt_ParamErr = linescan_Opt_ParamErr(:,:,1:numOpt);
    sumEigRelChange = sumEigRelChange(1:numOpt);

end

caliParam = linescan_Opt_Param(end, :);
tableOptimisedParameters = array2table(caliParam, 'VariableNames', paramHeaders);
disp(tableOptimisedParameters);

%% Project a single line-scan band onto the frame image of the pattern using the found calibration parameters (this is to check if the parameters are correct)

close all;

K1 = caliParam(9);
K2 = caliParam(10);
K3 = 0;
P1 = 0;
P2 = caliParam(11);
fy = caliParam(7);
v0 = caliParam(8);
t = caliParam(1:3);
rotEul = caliParam(4:6);
rotMat = eul2rotm(rotEul, 'ZYX');

%extrinsic of line-scan (frame camera coordinate frame w.r.t line-scan
%camera coordinate frame
T_F_2_LS = [rotMat, t'; 0, 0, 0, 1 ];
T_LS_2_F = T_F_2_LS\eye(4); %faster inverse

%figure for showing camera system
figCameras = figure('Name', 'Camera system');
plotCamera('Location', zeros(1,3), 'Orientation', eye(3), 'Size', 0.05, 'AxesVisible', true, 'Color', [1,0,0]); hold on;
plotCamera('Location', T_LS_2_F(1:3,4), 'Orientation', T_LS_2_F(1:3, 1:3)', 'Size', 0.05, 'AxesVisible', true, 'Color', [0,0,1]); hold on;
DisplayPattern(figCameras, pattern, linePtsTransFrame(:,:,1), extPosePattern(:,:,1), false);
title('RED: Frame camera (Origin)      BLUE: Line-scan camera');

xlabel('x');
ylabel('y');
zlabel('z');
grid on;
axis equal;
view([-115,-55]);

%image size (vertical line of pixels)
rowsHyp = numPixLS;
colsHyp = 1;

%intrinsic object for the linescan camera
lineScanIntr = cameraIntrinsics([fy, fy], [realmin,v0],[rowsHyp,colsHyp], 'RadialDistortion', [K1,K2,K3], 'TangentialDistortion', [P1,P2]);
lineScanCamParam = cameraParameters('IntrinsicMatrix', lineScanIntr.IntrinsicMatrix, 'ImageSize', [rowsHyp,colsHyp], 'RadialDistortion', [K1,K2,K3], 'TangentialDistortion', [P1,P2]);

%get intrinsic matrix (upper matrix)
K_ls = lineScanIntr.IntrinsicMatrix';
K_ls(1,3) = 0;

%single band we are visualising
hsBand = 100;

% undistort line-scan pixels and determine their normalised homography image coordinates
imgLinePixHom = 1:rowsHyp;
imgLinePixHom = [zeros(size(imgLinePixHom));imgLinePixHom]';
imgLinePixHom = undistortPoints(imgLinePixHom, lineScanCamParam)';
imgLinePixHom = [imgLinePixHom; ones(1,size(imgLinePixHom,2))];
normPtsLS = K_ls\imgLinePixHom;
numPts = size(normPtsLS, 2);

figCheckParam = figure('Name', 'Transform line-scan pixels onto the frame image of the pattern');

for i = 1:numImages
    curImg = imagesLS{i};
    pixelValues = double(curImg(hsBand, :));

    %transform board surface normal and point to line-scan coordinate
    %system
    T_pat_2_LS = T_F_2_LS*extPosePattern(:,:,i);
    tarSurfNorm = T_pat_2_LS(1:3,3)';
    ptOnTarget = T_pat_2_LS(1:3,4)';

    ptsFrame = zeros(numPts, 3);
    numPtsCount = 0;

    for ptLoop = 1:numPts
        normPtLS = normPtsLS(:,ptLoop)';
        %ray-trace LS pixel to board in LS coordinate frame
        [pntTargetLS, validIntersection] =  CameraPixelRayIntersectPlane(normPtLS, tarSurfNorm, ptOnTarget);

        %point does not intersect plane
        if ~validIntersection
            continue;
        end

        %transform ray-traced point to frame coordinate frame
        pntTargetFrHom = T_LS_2_F*[pntTargetLS, 1]';
        numPtsCount = numPtsCount + 1;
        ptsFrame(numPtsCount, :) = pntTargetFrHom(1:3)';
    end

    ptsFrame = ptsFrame(1:numPtsCount, :);

    %transformed points and their values
    ptsFramePixel = [ptsFrame,pixelValues'];

    %project transformed normalised line-scan points to the frame camera image
    lineImgPts = projectPoints(ptsFramePixel, kFrame, eye(4), [], frameImgSize);

    lsPatternPts = projectPoints(linePtsTransFrame(:,:,i), kFrame, eye(4), [], frameImgSize);

    %plot frame image
    clf(figCheckParam);
    imgCur = insertText(imagesFrame{i}, [0,0], 'x : Line-scan first pixel', 'BoxColor', "white", 'TextColor','red', 'BoxOpacity',1);
    imshow(imgCur); hold on;

    %separate projected image pixel coordinates
    u = lineImgPts(:,1)';
    v = lineImgPts(:,2)';
    w = zeros(size(u));
    %normalise pixel values based on min/max
    col = normalize(lineImgPts(:,3)', 'range');

    %store the normalised pixel values into a colour array
    colRGB = zeros(1,length(col),1);
    colRGB(:,:,1) = col;

    %plot the coloured line to the image
    plNaive = surface([u;u],[v;v], [w;w], [colRGB;colRGB],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2, 'DisplayName', 'Active', 'FaceColor', [0.9290 0.6940 0.1250]);
    colormap(jet);

    for j = 1:size(lsPatternPts,1)
       text(lsPatternPts(j,1), lsPatternPts(j,2), num2str(j), 'Color', [0,1,0]);
    end

    plot(u(1), v(1), 'rx');
    drawnow();
end


%% Saving to result directory

disp('Saving results ...');

resultFileName = fullfile(resultDir, 'camera_system_optimised_parameters.mat');

if ~exist(resultFileName, 'file')
    if optmAlgStruct.skipCov
        save(resultFileName, 'linescan_Opt_Param', 'numOpt', 'numPixLS');
    else
        save(resultFileName, 'linescan_Opt_ParamErr', 'linescan_Opt_Param', 'rankFIM', 'eigFIM', 'numOpt', 'sumEigRelChange', 'numPixLS');
    end
else
    userIn = input('Overwrite results Y/N?', 's');

    if lower(userIn) == 'y'
        if optmAlgStruct.skipCov
            save(resultFileName, 'linescan_Opt_Param', 'numOpt', 'numPixLS');
        elseif naiveCalibration
            save(resultFileName, 'linescan_Opt_Param', 'numOpt', 'linescan_Opt_ParamErr', 'numPixLS');
        else
            save(resultFileName, 'linescan_Opt_ParamErr', 'linescan_Opt_Param', 'rankFIM', 'eigFIM', 'numOpt', 'sumEigRelChange', 'numPixLS');
        end
    end
end
