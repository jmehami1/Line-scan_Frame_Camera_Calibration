function linePtsTransFrame_Cov = UncertaintyFrameCameraParam(pattern, lsPointsPatCoords, lsPointsPatCoordsERR, numImages, stdImgCoor, stdRGB_Intr, thetaFrameintr, distCoefCV, frameImgSize, imagesFrame, extPosePattern)
% Calculates the uncertainty in the line-scan 3D points introduced by the
% frame camera through its estimation in the pose of the calibration board.
% The unceratinty in the pose of the board get propagated to the
% uncertainty in the estimated location of the 3D line-scan points.
% INPUTS:
% 
% OUTPUT:
%       linePtsTransFrame_Cov - calculated covariance of each 3D line-scan point 
% Author:Jasprabhjit Mehami, 13446277


numLines = pattern.numLines;

%ArUco parameters
xNumMarker = pattern.NumCols;
yNumMarker = pattern.NumRows;
arucoLen = pattern.ArucoSideLength;
sepLen = pattern.SeparationLength;

%Number of unknown. This is from the transformation (3 rotation and 3 translation elements)
numTheta = 6;
numFrameInt = length(thetaFrameintr);

%rearrange the passed in distortion array
distCoef = [distCoefCV(1:2),distCoefCV(5),distCoefCV(3:4)];


% 2D marker corner positions in world coordinates (metres)
% 3D world coordinates that exist on the pattern
markerCornerCell = ArUcoBoardMarkerCornersCell(0, xNumMarker, yNumMarker, arucoLen, sepLen);
worldPoints = zeros(size(markerCornerCell,1)*4, 3);
for i = 1:size(markerCornerCell,1)
    curCorner =  markerCornerCell{i};
    worldPoints((i-1)*4 + 1: 4*i, 1:2) = curCorner;
end

% Covariance matrix of frame camera intrinsic parameter without
% distortion uncertainty
% ignore intrinsic error if very small
if all(stdRGB_Intr < 1e-4)
    sigmaFrameInt = thetaFrameintr;
    covWeight = 0;
    numSigmaPoints = 1;
else
    covFrameIntr = diag(stdRGB_Intr.^2);
    %Get sigma points from UT
    [sigmaFrameInt, ~, covWeight] = UnscentedTransformSigmaPoints(thetaFrameintr, covFrameIntr, numFrameInt, 4*numFrameInt, 0.5);
    numSigmaPoints = 2*numFrameInt+1;
end

linePtsTransFrame_Cov = zeros(3, 3, numLines, numImages);



for i = 1:numImages
    curExt = extPosePattern(:,:,i);
    eulExt = tform2eul(curExt, 'ZYX');
    T = tform2trvec(curExt);
    
    thetaFrame = [T, eulExt];
    %jacobian of pinhole frame camera w.r.t theta
    jac = JacobianFramePinholeOptReprojection(thetaFrame, [thetaFrameintr, distCoef], worldPoints);
    
    %covariance matrix of pixel feature points (diagional matrix)
    covU = diag((1/(stdImgCoor^2)).*ones(1,2*size(worldPoints,1)));
    %Calculate the covariance due to the pixel uncertainty
    covHZ_Frame = inv(jac'*covU*jac);
    
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
        cameraParams = cameraParameters('IntrinsicMatrix', K_matrix_, 'ImageSize', frameImgSize, 'RadialDistortion', distCoef(1:3), 'TangentialDistortion', distCoef(4:5));
        
        [rotMat, trans, found] = ArucoPosEst(imagesFrame{i}, markerCornerCell, cameraParams, false);        
        
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