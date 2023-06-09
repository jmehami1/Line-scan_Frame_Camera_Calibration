function [rotMat, trans, found, imgOut] = ArucoPosEst(img, xNumMarker, yNumMarker, arucoLen, sepLen, numMarkersExp, cameraParams)
%Estimate the extrinsic transformation of a Aruco board w.r.t to a frame
%camera. This Aruco board does not require all the markers to be present.


markerCornerCell = cell(yNumMarker*xNumMarker, 1);
ind = 0;

%Get 3D pattern position for each corner of each marker
for i = 1:yNumMarker
    for j = 1:xNumMarker
        x_tl = (j-1)*arucoLen + (j - 1)*sepLen;
        y_tl = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
        
        x_tr = j*arucoLen + (j - 1)*sepLen;
        y_tr = (yNumMarker - i + 1)*arucoLen + (yNumMarker - i)*sepLen;
        
        x_br = j*arucoLen + (j - 1)*sepLen;
        y_br = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
        
        x_bl = (j-1)*arucoLen + (j - 1)*sepLen;
        y_bl = (yNumMarker - i)*arucoLen + (yNumMarker - i)*sepLen;
        
        markerCorner = [
            x_tl, y_tl;
            x_tr, y_tr;
            x_br, y_br;
            x_bl, y_bl;
            ];
        
        ind = ind + 1;
        markerCornerCell(ind) = {markerCorner};
    end
end

img = undistortImage(img, cameraParams);


%find the ArUco tags in the image
[idsFound, markerCornFound, imgOut] = ArucoPixDect(img);

numMarkers = length(idsFound);

%if less than expected tags found, then skip image
if numMarkers < 0.2*numMarkersExp
    found = false;
    rotMat = eye(3);
    trans = zeros(1,3);
    return;
end


actMarkerCornerCell = cell(numMarkers, 1);

%only keep the 3D points markers which are in the markerID list
for i = 1:numMarkers
    actMarkerCornerCell(i) = markerCornerCell(idsFound(i)+1);
end

%convert the cell array into a matrix of 3D positions
markerPatPts = zeros(numMarkers*4, 2);

for i = 1:numMarkers
    curCorner =  actMarkerCornerCell{i};
    markerPatPts((i-1)*4 + 1: 4*i, :) = curCorner;
end


markerImgPts = zeros(4*numMarkers, 2);

%organise the corresponding image coordinates to match each marker corner
%to its 3D position.
for i = 1:numMarkers
    corners = markerCornFound(i,:);
    
    uPts = corners(1:2:end);
    vPts = corners(2:2:end);
    
    markerImgPts((i-1)*4 + 1 : 4*i, :) = [uPts', vPts'];
end

% markerImgPts = undistortPoints(markerImgPts, cameraParams);


%estimate extrinsic
[rotMat, trans] = extrinsics(markerImgPts, markerPatPts, cameraParams);


% K = cameraParams.IntrinsicMatrix';
% 
% 
%     %IPPE SOLUTION
%     imgPointsHom = [markerImgPts'; ones(1,markerImgPts)];
%     Q = K\imgPointsHom; %Apply intrinsic matrix to image coordinates
%     Q = Q(1:2,:);
%     
%     %Parameters for solver
%     homogMethod = 'Harker';
%     opts.measureTiming = false;
%     opts.withPoseRefinement = true;
%     
%     %Find the extrinsic parameters of each camera
%     [~,refinedPoses] = perspectiveIPPE(worldPoints',Q,homogMethod,opts);
%     R = refinedPoses.R1; %rotations of extrinsic (3x3)
%     T = (refinedPoses.t1)'; %translations of extrinsic (3x1)





found = true;

end

