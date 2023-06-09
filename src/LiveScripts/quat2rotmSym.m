function R = quat2rotmSym( q )
%This is the MATLAB script used for converting quaternion angle to a rotation
%matrix, but it has been edited to work with symbolic variables.
%Editor: Jasprabhjit Mehami, 13446277

%QUAT2ROTM Convert quaternion to rotation matrix
%   R = QUAT2ROTM(QOBJ) converts a quaternion object, QOBJ, into an orthonormal
%   rotation matrix, R. Each quaternion represents a 3D rotation. 
%   QOBJ is an N-element vector of quaternion objects.
%   The output, R, is an 3-by-3-by-N matrix containing N rotation matrices.
%
%   R = QUAT2ROTM(Q) converts a unit quaternion, Q, into an orthonormal
%   rotation matrix, R. The input, Q, is an N-by-4 matrix containing N quaternions. 
%   Each quaternion represents a 3D rotation and is of the form q = [w x y z], 
%   with w as the scalar number. Each element of Q must be a real number.
%
%   Example:
%      % Convert a quaternion to rotation matrix
%      q = [0.7071 0.7071 0 0];
%      R = quat2rotm(q)
%
%      % Convert a quaternion object
%      qobj = quaternion([sqrt(2)/2 0 sqrt(2)/2 0]);
%      R = quat2rotm(qobj);
%
%   See also rotm2quat, quaternion

%   Copyright 2014-2018 The MathWorks, Inc.

%#codegen

q = q/norm(q);

s = q(1);
x = q(2);
y = q(3);
z = q(4);

% Explicitly define concatenation dimension for codegen
tempR = cat(1, 1 - 2*(y.^2 + z.^2),   2*(x.*y - s.*z),   2*(x.*z + s.*y),...
2*(x.*y + s.*z), 1 - 2*(x.^2 + z.^2),   2*(y.*z - s.*x),...
2*(x.*z - s.*y),   2*(y.*z + s.*x), 1 - 2*(x.^2 + y.^2) );

R = reshape(tempR, [3, 3]);
R = permute(R, [2 1]);

end

