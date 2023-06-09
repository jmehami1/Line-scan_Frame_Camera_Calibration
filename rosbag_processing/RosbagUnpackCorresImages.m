% Unpack a rosbag which contains two image topics and find and save corresponding images based on closest time 
%
% Inputs:
%   rosbag - A MATLAB rosbag object.
%   topic1Name -  the name of first topic.
%   topic2Name -  the name of the second topic.
%   tdiff_max - maximum time difference allowed between corresponding
%               images
%   step_size - Get every step_size corresponding image (used to skip
%   images)
%
% Outputs:
%   topic1_Images - cell array of images from first topic
%   topic2_Images - cell array of images from second topic
%   numStored - number of images stored
%
% Author: Jasprabhjit Mehami, 13446277

function [topic1_Images, topic2_Images, numStored] = RosbagUnpackCorresImages(rosbag, topic1Name, topic2Name, tdiff_max, step_size)

topic2 = select(rosbag, 'Topic', topic2Name);
topic1 = select(rosbag, 'Topic', topic1Name);

%get number of messages (number of images)
numMes1 = topic1.NumMessages;
numMes2 = topic2.NumMessages;


if (numMes1 < 1)
    error([topic1Name, ' has no images']);
end

if (numMes2 < 1)
    error([topic2Name, ' has no images']);
end

%Get minimum number of messages (images) from both topics
%topic 2 has fewer images
if numMes1 > numMes2
    numMes = numMes2;
    topic2Fewer = true;
%topic 1 has fewer images
else
    numMes = numMes1;
    topic2Fewer = false;
end

%preallocate cell array outputs
topic1_Images = cell(1,numMes);
topic2_Images = cell(1,numMes);
numStored = 0;

%array of time differences used to display histgram of time differences
timeDiffArr = zeros(1,numMes);

% Go through such step_size frame and unpack
for i = 1:step_size:numMes
    
    fprintf('%d of %d...\n',i,numMes);
    
    
    if topic2Fewer
        topic2_Time = topic2.MessageList(i,:).Time;
        
        %find smallest difference between the current image of topic 2 and
        %all the images of topic 1
        [time_difference,index] = min(abs(topic1.MessageList{:,1}-topic2_Time));
        
    else
        topic1_Time = topic1.MessageList(i,:).Time;
        
        %find smallest difference between the current image of topic 1 and 
        %all the images of topic 2
        [time_difference,index] = min(abs(topic2.MessageList{:,1}-topic1_Time));
    end
    
    %store this time difference
    timeDiffArr(i) = time_difference;
    
    %check if the time difference is greater than allowed maximum
    if time_difference > tdiff_max
        continue;
    end
    
    %new image found. increment by 1.
    numStored = numStored +1;
    
    %Read corresponding image message from topics
    if topic2Fewer
        topic1_mesImg = readMessages(topic1, index);
        topic2_mesImg = readMessages(topic2, i);
    else
        topic1_mesImg = readMessages(topic1, i);
        topic2_mesImg = readMessages(topic2, index);
    end
    
    %save images into array
    topic1_Img = readImage(topic1_mesImg{1});
    topic1_Images{numStored} = topic1_Img;
    
    topic2_Img = readImage(topic2_mesImg{1});
    topic2_Images{numStored} = topic2_Img;
    
end

fprintf('Average time difference of %d\n', mean(timeDiffArr));

figure('Name', 'Time differences between images of topic 1 and topic 2');
histogram(timeDiffArr); hold on;
xlabel('Time (s)');

%clip cell arrays to correct size
topic2_Images = topic2_Images(1:numStored);
topic1_Images = topic1_Images(1:numStored);

end