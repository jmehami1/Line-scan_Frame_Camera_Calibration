function [cameraParams, K, distRad, distTan, imageSize, distCoefCV, fx, fy, u0, v0, std_f, std_u0v0] = loadcameraintrinsic(matFile)
%Loads the intrinsic parameters from a MATLAB cameraParameters structure
%stored inside of a MAT file.
% INPUTS:
%       matFile - MAT file with cameraParameters structure

if ~exist(matFile, 'file')
    error("%s not found", matFile);
end

matData = load(matFile);
cameraParams = matData.cameraParams;

%extract focal length components in pixels
fx = cameraParams.FocalLength(1);
fy = cameraParams.FocalLength(2);

%optical centre
u0 = cameraParams.PrincipalPoint(1);
v0 = cameraParams.PrincipalPoint(2);

%Size of the images in pixels[rows, columns]
imageSize = cameraParams.ImageSize;

%intrinsic object for the RGB camera
K = cameraParams.IntrinsicMatrix';

%distortion parameters
distRad = cameraParams.RadialDistortion;
distTan = cameraParams.TangentialDistortion;
distCoefCV = [distRad(1:2), distTan, distRad(3)]; %array of distortion coefficients in opencv format

%extract the error in the intrinsic parameters of the frame camera
% std_f = estimationErrors.IntrinsicsErrors.FocalLengthError;
% std_u0v0 = estimationErrors.IntrinsicsErrors.PrincipalPointError;
% stdRGB_Intr = [std_f,std_u0v0];
if exist('matData.estimationErrors', 'var')
    %extract the error in the intrinsic parameters of the frame camera
    std_f = matData.estimationErrors.IntrinsicsErrors.FocalLengthError;
    std_u0v0 = matData.estimationErrors.IntrinsicsErrors.PrincipalPointError;
else
    %assume no error in frame camera intrinsic
    std_f = [0, 0];
    std_u0v0 = [0, 0];
end

end