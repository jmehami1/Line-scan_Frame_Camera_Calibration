function [rotMat, trans, found, imgOut] = ArucoPosEst(img, markerCornerCell, cameraParams)
% Estimate the extrinsic pose of a ArUco planar board w.r.t to a frame
% camera (Transformation from world coordinates to camera coordinates). 
% This Aruco board can work under occluded or missing markers.
% INPUTS:
%       img - distorted image of ArUco board
%       markerCornerCell - cell array of 2D corner coordinates of each marker
%           in the coordinate frame of the board (world coordinates). All
%           corners have a Z-component of 0. This allows for custom boards
%           that don't require a full grid of markers.
%           [yNumMarker*xNumMarker x 1]
%       cameraParams - MATLAB struct of camera parameters
% OUTPUTS:
%       rotMat - rotation matrix of extrinsic[3 x 3]
%       trans - translation matrix of extrinsic [1 x 3]
%       found - flag if pose could be found
%       imgOut - image with outlined markers found. Same size as input image
% Author: Jasprabhjit Mehami, 13446277

%undistort image
imgUndistort = undistortImage(img, cameraParams);

%find the ArUco markers in the image
[idsFound, markerCornFound, imgOut] = ArucoPixDect(imgUndistort);

%actual number of markers found in image
numMarkersFound = length(idsFound);

%Need atleast two markers to be found
if numMarkersFound < 2
    found = false;
    rotMat = eye(3);
    trans = zeros(1,3);
    return;
end

%store the actual marker corners for the IDs found
actMarkerCornerCell = cell(numMarkersFound, 1);

%only keep the 3D points markers which are in the markerID list
for i = 1:numMarkersFound
    actMarkerCornerCell(i) = markerCornerCell(idsFound(i)+1);
end

%convert the cell array into a matrix of 2D positions on planar board
markerPatPts = zeros(numMarkersFound*4, 2);

for i = 1:numMarkersFound
    curCorner =  actMarkerCornerCell{i};
    markerPatPts((i-1)*4 + 1: 4*i, :) = curCorner;
end

markerImgPts = zeros(4*numMarkersFound, 2);

%organise the corresponding image coordinates to match each marker corner
%to its 2D position.
for i = 1:numMarkersFound
    corners = markerCornFound(i,:);
    
    uPts = corners(1:2:end);
    vPts = corners(2:2:end);
    
    markerImgPts((i-1)*4 + 1 : 4*i, :) = [uPts', vPts'];
end

%intrinsic matrix
K = cameraParams.IntrinsicMatrix';

%IPPE SOLUTION
imgPointsHom = [markerImgPts'; ones(1,size(markerImgPts,1))];
Q = K\imgPointsHom; %find normalised coordinates
Q = Q(1:2,:);

%Parameters for solver
homogMethod = 'Harker';
opts.measureTiming = false;
opts.withPoseRefinement = true;

%Find the refined extrinsic parameters of each pose of the board
[~,refinedPoses] = perspectiveIPPE(markerPatPts',Q,homogMethod,opts);
%IPPE gives two solutions, the first solution is the one with the smallest
%error
rotMat = refinedPoses.R1; %rotations of extrinsic (3x3)
trans = (refinedPoses.t1)'; %translations of extrinsic (3x1)

found = true;

end

