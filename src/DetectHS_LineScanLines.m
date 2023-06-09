function linePos = DetectHS_LineScanLines(img,numLines)
%Can automatically detect the v-pixel coordinates of the edges of the pattern in the
%line-scan image. It may require user input if the correct number of lines
%are not found. This usually happens when artifacts which are not pattern
%are seen in the field of view of the camera.
%Inputs:
%       img - linescan image wheres rows represent bands
%       numLines - number of expected lines from the image
%Output:
%       linePos - v-Position of detected lines along the image
%Author: Jasprabhjit Mehami, 13446277

[bands, numPixels] = size(img);

if bands < 256
    sumBands = sum(img,1)';
else
    
    %upper and lower limits bands from the HS image
    bandLower = 50;
    bandUpper = 190;
    
    %Sum the columns of the HS image
    sumBands = sum(img(bandLower:bandUpper,:), 1)';
    
end

%find the locations of the peaks. It tries to find the correct peaks
locs = FindPeaksOfSum(sumBands, numPixels);

% check if the correct number of lines is found. If not, there is likely other artifacts in
% in the image. Need to specific location of end points of pattern and repeat search.
% if length(locs) ~= numLines
%     fig = figure('Name','Pick start and end points of the pattern in the image');
%     imshow(img);
%     fig.WindowState = 'maximized';
%     refresh(fig);
%     [x,~] = getpts(fig);
%
%     %location of start and end points of the pattern selected by user
%     patStart = round(x(1));
%     patEnd = round(x(2));
%
%     %Sum the columns of the HS image
%     sumBands = sum(img(bandLower:bandUpper,:), 1)';
%     %Set everything outside start and end to be equal to the values at
%     %start and end
%     sumBands(1:patStart-1) = sumBands(patStart);
%     sumBands(patEnd-1:end) = sumBands(patEnd);
%
%     %find the locations of the peaks. It tries to find the correct peaks
%     locs = FindPeaksOfSum(sumBands, numPixels);
%
%     close(fig);
% end

% If still can't find correct number then image is not valid.
if length(locs) ~= numLines
    linePos = -1;
    return;
end

%extra refinement
for i = 1:length(locs)
    locs(i) = RefinePeaks(locs(i), sumBands);
end

linePos = locs;

end

%find the peaks of a 1D array which contains of the pixel bands summed
%across the hyperspectral image
function locs = FindPeaksOfSum(sumBands, numPixels)
% figure(1);
% plot(sumBands);

%apply smoothing to sum
sumBandsSmooth = smooth(sumBands,3)';

% figure(2);
% plot(sumBandsSmooth);

%get the gradient in the x direction (along the row) of the sum
gx = imgradientxy(sumBandsSmooth, 'intermediate');

% figure(3);
% plot(gx);

%find peaks along gradient of sum
[~, locsPos] = findpeaks(gx, 'MinPeakProminence', max(gx)/3);
% findpeaks(gx, 'MinPeakProminence', max(gx)/3);

%small correction of 1 pixel added to peaks
% locsPos = locsPos + 1;

% figure(4);
% plot(-gx);

%find troughs along gradient of sum
[~, locsNeg] = findpeaks(-gx, 'MinPeakProminence', abs(min(gx)/3));
% findpeaks(-gx, 'MinPeakProminence', abs(min(gx)/3));

%small correction of 1 pixel subtracted to troughs
% locsNeg = locsNeg - 1;

%make sure the gradients of the peaks and troughs follow the required
%pattern
locs = CheckPeakSequence(locsPos, locsNeg, numPixels);

end

%helper function for extra refinement.
function locRefine = RefinePeaks(loc, sumBands)

sumVal = sumBands(loc);
sumValPrev = sumBands(loc-1);
sumValNext = sumBands(loc+1);

%Peak is on the right (closer to the next position)
if sumVal < sumValNext && sumVal > sumValPrev
    locRefine = (sumVal*loc + sumValNext*(loc+1))/(sumVal+sumValNext);
    %Peak is on the left (closer to the previous position)
elseif sumVal < sumValNext && sumVal > sumValPrev
    locRefine = (sumVal*loc + sumValPrev*(loc-1))/(sumVal+sumValPrev);
else
    locRefine = loc;
end

end

%Tries to find the correct sequences of peaks and troughs in the
%hyperspectral image
function locsCorrPeaks = CheckPeakSequence(locsPos, locsNeg, numPixels)

locs = [locsPos, locsNeg]; %combine peaks and throughs into single array

%this is a array which is the same size as the locs and tells us whether
%that location was a peak or a trough
peaksPatternSeqence = [ones(1,length(locsPos)), -ones(1,length(locsNeg))];

%Remove any peaks that are at pixels 2 or less and numPixels -1 or higher
peaksPatternSeqence = peaksPatternSeqence(locs > 2);
locs = locs(locs>2);
peaksPatternSeqence = peaksPatternSeqence(locs < (numPixels-1));
locs = locs(locs<numPixels-1);

%remove all repeated elements
[~, i1] = unique(locs,'first');
[locs, i2] = unique(locs,'last');
locs = locs(i1==i2);%b(i1==i2);
peaksPatternSeqence = peaksPatternSeqence(i2);
peaksPatternSeqence = peaksPatternSeqence(i1==i2);

%sort peaks in ascending
[locsSort, indexes] = sort(locs, 'ascend');
peaksPatternSeqence = peaksPatternSeqence(indexes);

peaksPattern = [1,-1,1,-1,1,-1,1,-1];

%find the pattern in the sequence
startPos = strfind(peaksPatternSeqence, peaksPattern);

%Check if the startPos is empty and length of startPos is greater than 1
if isempty(startPos) || length(startPos) > 1
    locsCorrPeaks = -1;
    return;
end

%copy from startPos to 7 elements later (there should be 8)
locsCorrPeaks = locsSort(startPos:startPos+7);

end
