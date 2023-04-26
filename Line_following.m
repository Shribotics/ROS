
%After connetong to the turtlebot 
%virtul machine IP address
%ip_Turtlrbot_vm = "http://192.168.146.129:11311" ;
ip_Turtlrbot_vm = "http://192.168.146.129:11311" ;
% start ROS master
rosinit(ip_Turtlrbot_vm)

%subscribe to the came
% ra image
sub2 = rossubscriber('/camera/image/compressed',"sensor_msgs/CompressedImage");
%publich the cmd_vel node to control the robot
velPub = rospublisher("/cmd_vel","geometry_msgs/Twist") ;
pause(1);

%defininf the PID gains
Kp = 0.05;
Ki = 0.001;
Kd = 0.1;

% PID controller variables
error = 0;           % Error between desired and actual position
errorPrev = 0;       % Error in previous iteration
errorIntegral = 0;   % Accumulated error over time
%maximum velocity 
vel_max = 0.2;
%min velocity
vel_min = 0.02;

%define the angular velocity
angular_max = 0.84;

%define the angular valocity minimum
angular_min = -0.84;

%Moving turtle bot for frist 1.5 seconds to know about the lines
velMsg = rosmessage(velPub);
velMsg.Linear.X = 0.1;
duration = 1.5; % Set duration of movement in seconds
start_time = rostime('now');
while rostime('now') - start_time < duration
    send(velPub, velMsg); % Publish Twist message
    pause(0.1); % Wait for 0.1 seconds
end

% Stop the robot
velMsg.Linear.X = 0; % Set linear velocity to 0
send(velPub, velMsg); % Publish Twist message to stop the robot

while true
    velMsg = rosmessage(velPub);
    % Receive camera image
    cameraMsg = receive(sub2);
    img = readImage(cameraMsg);
    image = img(210:240,1:320,:);
    
    % Convert image from RGB to LAB color space
    labImage = rgb2lab(image);
    
    % Define range of yellow color in a-channel
    aLow = 0;
    aHigh = 128;
    
    % Threshold image to extract yellow pixels
    yellowMask = (labImage(:,:,2) >= aLow) & (labImage(:,:,2) <= aHigh);
    
    % Morphological operations to clean up binary mask
    se = strel('disk', 5);
    yellowMask = imclose(yellowMask, se);
    yellowMask = imfill(yellowMask, 'holes');
    yellowMask = imopen(yellowMask, se);

    %show the original image, masked image and yellow line detection
    figure(1);
    subplot(221);
    imshow(image)
    subplot(222)
    imshow(yellowMask)
    subplot(223)
    imshow(labImage)
    
    % Get the co-ordinated for yellow pixel line
    [yellowY, yellowX] = find(yellowMask);

    % Compute center  of yellow pixels
    cX = mean(yellowX);
    cY = mean(yellowY);
    %show the position of the center of yellow line to follow
    figure(1);
    subplot(224)
    plot(cX,cY,'.',LineWidth=20)
  
     % Calculate error to keep the centre of yellow line in the middle of
     % the lines
     error = cX - size(image, 2)/2;
     pause(0.1)
    %if the error is zero then do not rotate the robot and move robot at
    %highest velocity
    %pause for one second to know the error is 0
     if isequal(error,0)
         pause(1)
         velMsg.Linear.X = vel_max;
         velMsg.Angular.Z = 0;
         send(velPub, velMsg);
     else
         %else use the PID controller to get the robot at the centre of the
         %line
         pTerm = Kp * error;
%         
%         % Calculate integral term
         errorIntegral = errorIntegral + error;
         iTerm = Ki*errorIntegral;
    %         
    %    % Calculate derivative term
         dTerm = Kd * (error - errorPrev);
         errorPrev = error;
    %         
    %         % Calculate control output
         controlOutput = pTerm + iTerm + dTerm;       
         
         %control the angle of rotation and kep it within the defined
         %limits of -0.84 and 0.84
         controlOut_angle = max(min(controlOutput, angular_max), angular_min);
         angular =  controlOut_angle;
    %    % Control robot
         velMsg.Linear.X = vel_min;
         velMsg.Angular.Z = angular;
         send(velPub, velMsg);
     end
     
        
     % Wait for a short period of time to allow the robot to move
     pause(0.1);
 
end
