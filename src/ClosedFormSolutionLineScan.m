function phi0 = ClosedFormSolutionLineScan(worldCoord, v, t3Act, t1Act)
%Closed form solution to the line-scan and Frame camera system calibration
%(Does not calculate distortion)
%INPUTS:
%   worldCoord - world coordinates of the feature points
%   v - line-scan image coordinate of the feature points in the coordinate
%       frame of the frame camera
%   t3Act - Approximate displacement of the frame camera w.r.t line-scan
%   camera along the z-direction (forward-backwards)
%   t1Act - Approximate displacement of the frame camera w.r.t line-scan
%   camera along the x-direction (left-right)

%OUTPUT:
%   phi0 - closed form solution (initial guess to optimisation)
%Author:Jasprabhjit Mehami, 13446277

%World points seen by the line-scan camera in the coordinate frame of the
%frame camera
X = worldCoord(:,1);
Y = worldCoord(:,2);
Z = worldCoord(:,3);

%Padded ones vector the same size as the other vectors
onesMat = ones(length(X),1);

A1= [ X, Y, Z, onesMat];
A23 = [ X, Y, Z, onesMat, -v.*X, -v.*Y, -v.*Z, -v ];

%Solving the first equation of the system
[~,~,V1] = svd(A1);
M1 = V1(:,end);

%Solving the second equation of the system
[~,~,V23] = svd(A23);
M23 = V23(:, end);

%extracting vectors from SVD solutions
n1 = M1(1:3);
n2 = M23(1:3);
n3 = M23(5:7);

%Scale factor for n3
gam3 = 1/norm(n3);
t3 = gam3*M23(8);

%Gam3 has TWO solutions, the correct one needs to be chosen depending on
%whether the Frame camera is in front or behind the line-scan camera
%change to sign function instead
if abs(t3 - t3Act) > abs(-t3 - t3Act)
    gam3 = -gam3;
    t3 = -t3;
end

%scale factof for n1
fy = abs(norm((gam3^2)*cross(n2,n3)));
gam1 = 1/norm(n1./fy);
t1 = gam1*M1(4);

%Gam1 has TWO solutions, the correct one needs to be chosen depending on
%whether the Frame camera is on the left or right to the line-scan camera
if abs(t1 - t1Act) > abs(-t1 - t1Act)
    gam1 = -gam1;
    t1 = -t1;
end

%Intrinsic parameters
v0 = abs((gam3^2)*dot(n2,n3));

%calculate the row matrices of the rotation matrix
r3 = gam3*n3;
r1 = gam1.*n1;
r1 = normalize(r1, 'norm');
r2 = -cross(r1,r3);

%translation in y direction
t2 = (gam3*M23(4)-v0*t3)/fy;

%rotation matrix using row vectors
rotMatrix = [r1'; r2'; r3'];

% There are issues when optimising which result in the cameras facing the
% oppoosite direction. Assuming that cameras have overlapping views, both
% line-scan and frame camera should be facing the same direction. We check
% this using the dot product of z-direction unit vector.
% if dot product is negative, convert rotation matrix to euler angles,
% set the last angle as zero, and convert back to rotation matrix 
if dot(rotMatrix(:,3), [0,0,1]) < 0
    rot = rotm2eul(rotMatrix, 'zyx');
    rot(3) = 0; 
    rotMatrix =  eul2rotm(rot, 'zyx');  
end

%Convert to rotation matrix to quaternion
rot = rotm2quat(rotMatrix);

phi0 = [ t1, t2, t3, rot, fy, v0 ];

end

