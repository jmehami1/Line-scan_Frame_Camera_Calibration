function [linePtsTransFrame, linePtsTransFrame_Cov, extPosePattern, numImages, lsPointsImgCoords] = FrameCameraLineScanPointTransform(pattern, lsPointsImgCoords,numImages, stdImgCoor, stdRGB_Intr, thetaFrameintr, distCoefCV, frameImgSize, imagesRGB, displayOn)
%Combined functionality of determining the pattern's pose using the frame
%camera and determining the world points seen by the line-scan camera on the
%pattern. These world points will be transformed to a common coordinate frame
%of the frame camera. Pixel uncertainty and frame camera intrinsic
%uncertainty will be propagated to the final world points.
%INPUTS:
%       pattern - object defining the dimensions and parameters of the pattern
%       crossMarkerImgCoords - pixel coordinates of the cross-markers on
%           each of the supplied images
%       lsPointsImgCoords - line-scan pixel coordinates of the feature
%           points seen by the camera
%       crossMarkerPatCoords - World coordinates of the cross-markers in
%           the pattern's coordinate frame
%       numImages - number of supplied images
%       stdImgCoor - STD in the location of the pixel coordinates
%       stdRGB_Intr - STD in the value of the intrinsic parameters of the
%           frame camera
%       thetaFrameintr - Intrinsic parameters of the frame camera
%           [fx,fy,u0,v0]. Does not include distortion parameters
%       distCoef - distortion coefficients of the frame camera.
%           [K1,K2,K3,P1,P2]
%       sizeFrameImage - size of image. [rows,columns];
%       std_distCoef - [OPTIONAL] STD in distortion coefficient values of
%           the frame camera.
%OUTPUTS:
%       linePtsTransFrame - feature points seen by the line-scan camera in
%           in the frame camera's coordinate frame
%       linePtsTransFrame_Cov - Covariance matrix of each feature point
%           seen by the line-scan camera in the frame camera's
%           coordinate frame
%       extFrameAll - extrinsic transformation of all poses relative to the
%           frame camera's coordinate frame
%Author:Jasprabhjit Mehami, 13446277


numLines = pattern.numLines;

%ChArUco pattern size
xNumCheck = pattern.NumCols;
yNumCheck = pattern.NumRows;
checkSize = pattern.CheckerSideLength;
arucoSize = pattern.ArucoSideLength;

%intrinsic object for the RGB camera
frameIntrinsic = cameraIntrinsics(thetaFrameintr(1:2),thetaFrameintr(3:4), frameImgSize);
K_matrix = frameIntrinsic.IntrinsicMatrix;
numFrameInt = length(thetaFrameintr);

worldPoints =  generateCheckerboardPoints([yNumCheck,xNumCheck], checkSize);

%distCoefCV = [distRad(1:2), distTan, distRad(3)]; %array of distortion coefficients in opencv format
distCoef = [distCoefCV(1:2),distCoefCV(5),distCoefCV(3:4)];


extPosePattern = zeros(4,4,numImages);

goodImages = zeros(1,numImages);
numGoodImg = 0;

for i = 1:numImages
    [rotMat, trans, found, img] = CharucoPosEst(imagesRGB{i}, K_matrix, distCoefCV, ...
        xNumCheck, yNumCheck, checkSize, arucoSize);
    
    if ~found
        continue;
    end
    
    
    numGoodImg = numGoodImg + 1;
    goodImages(numGoodImg) = i;
    
    extPosePattern(:,:,numGoodImg) = [rotMat,trans'; 0, 0, 0, 1];
    
    if displayOn
        fig = figure(1);
        clf(fig);
        imshow(img); hold on;
    end
end

goodImages = goodImages(1:numGoodImg);
lsPointsImgCoords = lsPointsImgCoords(:,goodImages);
imagesRGB = imagesRGB(goodImages);
extPosePattern = extPosePattern(:,:,1:numGoodImg);
numImages = numGoodImg;


%% Find the linescan intersection points in the pattern coordinate system
lsPointsPatCoords = cell(1,numImages);
lsPointsPatCoordsERR = cell(1,numImages);

fig = figure('Name', 'test');

for i = 1:numImages
    [lsPointsPatCoords{i}, lsPointsPatCoordsERR{i}] = CrossRatioPatternPoints(pattern, lsPointsImgCoords(:,i), stdImgCoor^2);
    DisplayPattern(fig, pattern, lsPointsPatCoords{i}, eye(4));
    clf(fig);
end

%% Calibrate the frame camera using checkerpoints and transform points from pattern to frame camera coordinate frame

linePtsTransFrame =  zeros(numLines,3, numImages);

%Number of unknown. This is from the transformation (3 rotation and 3
%translation elements)
numTheta = 6;

linePtsTransFrame_Cov = zeros(3, 3, numLines, numImages);


for i = 1:numImages
    
    curExt = extPosePattern(:,:,i);
    eulExt = tform2eul(curExt, 'ZYX');
    T = tform2trvec(curExt);
    
    thetaFrame = [T, eulExt];
    %jacobian of pinhole frame camera w.r.t theta
    jac = JacobianPinholeFrameCamera(thetaFrame, [thetaFrameintr, distCoef], worldPoints);
    
    %covariance matrix of pixel feature points (diagional matrix)
    covU = diag((1/(stdImgCoor^2)).*ones(1,2*size(worldPoints,1)));
    %Calculate the covariance due to the pixel uncertainty
    covHZ_Frame = inv(jac'*covU*jac);
    
    
    %Covariance matrix of frame camera intrinsic parameter without
    %distortion uncertainty
    covFrameIntr = diag(stdRGB_Intr.^2);
    %Get sigma points from UT
    [sigmaFrameInt, ~, covWeight] = UnscentedTransformSigmaPoints(thetaFrameintr, covFrameIntr, numFrameInt, 4*numFrameInt, 0.5);
    
    numSigmaPoints = 2*numFrameInt+1;
    
    %determine the extrinsics from each set of intrinsic parameters for
    %the RGB camera
    extFrame_UT = zeros(numSigmaPoints, numTheta);
    
    %first sigma point is always the mean value
    extFrame_UT(1,:) = thetaFrame;
    
    
    for j = 2:numSigmaPoints
        %New sigma points
        fx_ = sigmaFrameInt(j,1);
        fy_ = sigmaFrameInt(j,2);
        u0_ = sigmaFrameInt(j,3);
        v0_ = sigmaFrameInt(j,4);
        
        frameIntrinsic_ = cameraIntrinsics([fx_, fy_],[u0_,v0_], frameImgSize);
        K_matrix_ = frameIntrinsic_.IntrinsicMatrix;
        
        
        [rotMat, trans, found] = CharucoPosEst(imagesRGB{i}, K_matrix_, distCoefCV, ...
            xNumCheck, yNumCheck, checkSize, arucoSize);
        
        if found
            eulExt = rotm2eul(rotMat, 'ZYX');
            extFrame_UT(j,:) = [trans, eulExt];
        end
    end
    
    
    %Covariance due to intrinsic parameters from UT run
    covUT_Frame = zeros(numTheta);
    
    %compute covariance of all runs from UT
    for j = 1:numSigmaPoints
        covUT_Frame = covUT_Frame + covWeight(j).*(extFrame_UT(j,:)-thetaFrame)'*(extFrame_UT(j,:)-thetaFrame);
    end
    
    %total error introduced by frame camera from each image
    covFrame = covUT_Frame + covHZ_Frame;
    
    %Turn feature points on pattern into homogenous coordinates
    linePtsPattern = [[lsPointsPatCoords{i}]';ones(1,numLines)];
    
    %Transform the points from the pattern to the frame coordinate frame
    ptsTrans = curExt*linePtsPattern;
    linePtsTransFrame(:,:,i) = ptsTrans(1:3,:)';
    
    %Uncertainty in the line-scan points on pattern
    linePtsPatternERR = [lsPointsPatCoordsERR{i}]';
    
    %Loop through each feature point from the current pose, and calculate
    %the uncertainty when transforming the point from the pattern to the
    %frame camera.
    for j = 1:numLines
        curPoint = linePtsPattern(1:3, j);
        curPointERR = linePtsPatternERR(:,j)';
        
        %Form the covariance matrix of theta and XYZ point component
        %Propagate the errors from the frame camera and the points on the
        %pattern
        covXYZ = diag(curPointERR);
        covXYZ_ThetaRGB = zeros(9);
        covXYZ_ThetaRGB(1:6,1:6) = covFrame;
        covXYZ_ThetaRGB(7:end, 7:end) = covXYZ;
        
        jac = JacobianTransPat2Frame(thetaFrame, curPoint');
        
        %Propagated covariance matrix to transformed points
        covTransPts = jac*covXYZ_ThetaRGB*jac';
        linePtsTransFrame_Cov(:,:,j,i) = covTransPts;
    end
end
end
