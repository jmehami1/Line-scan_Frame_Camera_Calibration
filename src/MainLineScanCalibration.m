function [linescan_Opt_Param, linescan_Opt_ParamErr, eigFIM, rankFIM, res] = MainLineScanCalibration(lsPointsImgCoords, linePtsTransRGB, linePtsTransRGB_ERR, stdImgCoor, numImages, pattern, t3Act, t1Act, optmAlgStruct)
% Main function for calibrating the line-scan camera system where the
% parameters return an associated uncertainty. Fisher information matrix is
% also calculated to determine the current level of information that the
% measurements provide.
%
% INPUTS:
%       lsPointsImgCoords - line-scan feature pixel points for all poses
%       linePtsTransRGB - line-scan feature points seen on the pattern
%           transformed into the coordinate frame of the frame camera
%       linePtsTransRGB_ERR - Covariance matrix for each transformed feature point
%           for each pose of the pattern
%       stdImgCoor -  STD of the image pixel location
%       numImages -  number of images/poses of pattern
%       pattern - object describing the pattern
%       t3Act - expected value of 't3'(Z axis). Is necessary to ensure optimisation
%           does not converge to mirror pose
%       t1Act -  expected value of 't1' (X axis). Is necessary to ensure optimisation
%           does not converge to mirror pose
% OUTPUTS:
%       linescan_Opt_Param - optimised camera parameters
%       linescan_Opt_ParamErr - covariance matrix of optimised parameters
%       eigFIM -  eigen values of the fisher information matrix
%       rankFIM - rank of the fisher information matrix
%
% Author: Jasprabhjit Mehami, 13446277
warning('off');
numLines = pattern.numLines;
numThetaLS = 11;

%Extract components of feature points and store in vector
X = reshape(linePtsTransRGB(:,1,:),[],1);
Y = reshape(linePtsTransRGB(:,2,:),[],1);
Z = reshape(linePtsTransRGB(:,3,:),[],1);
%Extract pixel feature points and store in vector
v = reshape(lsPointsImgCoords,[],1);

%calculate the initial guess of parameters using closed form solution
phi0_nodist = ClosedFormSolutionLineScan([X,Y,Z],v, t3Act, t1Act);

%distortion parameters of line-scan camera set to zero
k1 = 0;
k2 = 0;
p2 = 0;
phi0 = [phi0_nodist,k1,k2,p2];

%carry out optimisation
[thetaLS_quat, res] = LineScanCalibration([X,Y,Z], v, phi0, optmAlgStruct);

fprintf('Average pixel error per point: %d\n', res);

%convert optimised quaternion angle to euler
eulAVG = quat2eul(thetaLS_quat(4:7), 'ZYX');
thetaLS = [thetaLS_quat(1:3), eulAVG, thetaLS_quat(8:end)];
linescan_Opt_Param = thetaLS;

if optmAlgStruct.skipCov
    linescan_Opt_ParamErr = 0;
    eigFIM = 0;
    rankFIM = 0;
    return;
end


%jacobian of line-scan pinhole model w.r.t to line-scan parameters
jac = JacobianLinescanPinholeOptReprojection(thetaLS, [X,Y,Z]);

cov = 1/(stdImgCoor^2).*eye(size(jac,1));
%error caused by back propagation of features
covHZ_LS = inv(jac'*cov*jac);

%Total number of components for all world points
numWorldPts = numLines*3*numImages;

%Convert the matrix of worldpoints seen by the line-scan camera to a row
%vector
worldPtsRow = zeros(1, numWorldPts);
%Create a large covariance matrix of the world points
cov_worldPtsRow = zeros(numWorldPts);

%turn the world points into a single row array
for i = 1:numImages
    for j = 1:numLines
        worldPtsRow((i-1)*numLines*3 + (j-1)*3 + 1:(i-1)*numLines*3 + j*3) = linePtsTransRGB(j,:,i);
        %Create a single large covariance matrix of the uncertainty in
        %the world point components
        cov_worldPtsRow((i-1)*numLines*3 + (j-1)*3 + 1:(i-1)*numLines*3 + j*3,(i-1)*numLines*3 + (j-1)*3 + 1:(i-1)*numLines*3 + j*3) = linePtsTransRGB_ERR(:,:,j,i);
    end
end

%Get the sigma points of the world coordinates. This is basically a small
%change in 1 component of the world coorinates, which after re-running
%optimisation will return different parameters.
%mean weight is not used here because we assume that the mean is the same
%as the optimised parameters. (normally you would recalculate the mean from
%all runs of each sigma point, but this optimisation is not gaussian, so
%the mean of all runs is incorrect)
[sigmaWorldPts, ~, covWeight] = UnscentedTransformSigmaPoints(worldPtsRow, cov_worldPtsRow, numWorldPts/3, 4*numWorldPts, 0.5);

numSigmaPts = size(sigmaWorldPts,1);

%recalibrate the cameras using the calculated world coordinates
%from the sigma points
theta_UT = zeros(numSigmaPts, numThetaLS);
%start from 2 because the first row is the world coordinates
%already used to calibrate the camera (the mean)

%First sigma point is always the mean
theta_UT(1,:) = thetaLS;

%Using each sigma points of the world coordinates, run the optimisation to
%calculate the optimised parameters
for i = 2:numSigmaPts
    curWorldPtsRow = sigmaWorldPts(i,:); %current world points in row form

    %current world points in matrix form for calibration
    curlinePtsTransRGB = zeros(numLines, 3, numImages);

    %separate the world coordinates by images and then
    %coordinates
    for j = 1:numImages
        for k = 1:numLines
            curlinePtsTransRGB(k,:,j) = curWorldPtsRow((j-1)*numLines*3 + (k-1)*3 + 1:(j-1)*numLines*3 + k*3);
        end
    end

    %separate the components of the world points
    X = reshape(curlinePtsTransRGB(:,1,:),[],1);
    Y = reshape(curlinePtsTransRGB(:,2,:),[],1);
    Z = reshape(curlinePtsTransRGB(:,3,:),[],1);

    %The expected optimisation should be close to the already calculated
    %optimisation. Assume that the initial guess is the already calculated
    %optimised paramters
    phi0 = thetaLS_quat;

    %perform optimisation with current sigma point
    cur_theta_UT = LineScanCalibration([X,Y,Z],v, phi0, optmAlgStruct);

    %Convert quaternion to euler and store optimised parameter
    eul_UT = quat2eul(real(cur_theta_UT(4:7)), 'ZYX');
    theta_UT(i,:) = [cur_theta_UT(1:3), eul_UT, cur_theta_UT(8:end)];
end

%covariance matrix calculated from the unscented transform
covUT_LS = zeros(numThetaLS);


mean_thetaUT = thetaLS;

%compute covariance of all runs from UT
for j = 1:numSigmaPts
    covUT_LS = covUT_LS + covWeight(j).*(theta_UT(j,:)-mean_thetaUT)'*(theta_UT(j,:)-mean_thetaUT);
end

%Total uncertainty in the calibrated parameters
covTotalLS = covUT_LS + covHZ_LS;
linescan_Opt_ParamErr = covTotalLS;

%fisher information matrix requires the measurements be setup as an
%obseration equation. The observations come from both the
%linescan camera and the frame camera. Even though we are only
%calibrating the linescan camera, the frame camera induces its
%uncertainty in the world coordinates. This needs to be added to the
%observation equation.
covObsXYZ = zeros(numLines*2*numImages);

%populate the covariance matrix of the measurements, introduced
%by the frame camera uncertainty, by propagating the uncertainty of each
%set of world points through the line-scan pinhole camera model
for i = 1:numImages
    for j = 1:numLines
        cur_covXYZ = cov_worldPtsRow((i-1)*numLines*3 + (j-1)*3 + 1:(i-1)*numLines*3 + j*3,(i-1)*numLines*3 + (j-1)*3 + 1:(i-1)*numLines*3 + j*3);

        %Jacobian of the pinhole line-scan model w.r.t to the XYZ world
        %points seen by the line-scan camera in the coordinate frame of the
        %frame camera
        jacPinholeXYZ = JacobianLinescanPinholeOptReprojectionXYZ(thetaLS,linePtsTransRGB(j,:,i));

        covXYZ2uv = jacPinholeXYZ*cur_covXYZ*jacPinholeXYZ';

        %Add the propagated covariance matrix to a large covariance
        % matrix for all points
        covObsXYZ((i-1)*numLines + (j-1) + 1,(i-1)*numLines + (j-1) + 1) = covXYZ2uv(1,1);
        covObsXYZ((i-1)*numLines + 2*numLines + (j-1) + 1,(i-1)*numLines + 2*numLines + (j-1) + 1) = covXYZ2uv(2,2);

        covObsXYZ((i-1)*numLines + (j-1) + 1,(i-1)*numLines + 2*numLines + (j-1) + 1) = covXYZ2uv(1,2);
        covObsXYZ((i-1)*numLines + 2*numLines + (j-1) + 1,(i-1)*numLines + (j-1) + 1) = covXYZ2uv(2,1);
    end
end

%Total measurement uncertainty is from the world points and pixel
%coordinates
covObsTotal = covObsXYZ + diag(ones(1,numLines*2*numImages).*(stdImgCoor.^2));

%calculate the fisher information matrix (used for observability study and
%active calibration)
fimObser = (jac'/covObsTotal)*jac;

%calculate the rank and the eigenvalues of FIM (used for active
%calibration)
rankFIM = rank(fimObser, 1e-5);
eigFIM = eig(fimObser);
warning('on');

end

