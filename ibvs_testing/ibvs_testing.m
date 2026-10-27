% MATLAB controller for Webots
% File:          ibvs_testing.m
% Date:
% Description:
% Author:
% Modifications:
function ibvs_testing

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;
%Can be updated to store in files dynamically I think...
%maybe need to mkdir on fly tho, idk how to do that rn, if I have spare time...
file_path = "C:\Users\Kyle\Documents\Uni\Final year\FYP\Data\Plots\lum 3\Line\realisms\slip\";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NB!!!! UPDATE BLOCK PARAMS IN BLOCK MOVER CONTROLLER AS WELL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TIME_STEP = 64;
Block_speed = 0.08;
Block_Length = 1;
gain = 2;
sim_duration = 51.2;
needed = 0;
Approach = 0;
if Approach
  sim_duration = 10;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Vehicle Params and other constants
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%2WR parameters
d = 0.052; %m
r = 0.0205; %m
%l = d;

%velocity limits
phi_lim = 6.279;
a = r*phi_lim;
b = 2*r/d*phi_lim;
 
%Slip Ratios
sL = 0.3762;
sR = sL;

%slip calcs
sL_calc = 1 - sL;
sR_calc = 1 - sR;

%dead band tolerances
d_bnd_tol = 0.5; %pixels

%matching penalty for matchpairs function
costUnmatched = 15;

%%%%%%%%%%%%%%$
% SIM TOGGLES %
%%%%%%%%%%%%%%%
ON = 1;
OFF = 0;

%Path Shape
Straightline = ON;

% Realism Toggles
slip_toggle = ON; %toggles slip kinematic mapping
d_bnd_toggle = OFF; %dead band toggle

%plot toggles
plot_img = ON;
plot_error = ON;
plot_wheel_speeds = ON;
plot_points = ON;
plot_Jp = ON;
norm_e_plot = ON;
plot_loss = ON;

%sim duration

if Straightline
    dir_changes = 2; %The points where the block changes direction
    Perimeter = Block_Length * dir_changes;
    corner_time = Block_Length/Block_speed;
    sim_duration = Perimeter/Block_speed %seconds
else
    dir_changes = 4; %The points where the block changes direction
    Perimeter = Block_Length * dir_changes;
    corner_time = Block_Length/Block_speed;
    sim_duration = Perimeter/Block_speed %seconds
end
 

%sim_duration = 10 %seconds

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CAMERA ENABLE AND PAREMETER COLLECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get and enable devices
%enable camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP);
cam_node = wb_supervisor_node_get_from_device(camera)
tarbot_node = wb_supervisor_node_get_from_def('tarbot');
wb_supervisor_node_set_visibility(tarbot_node,cam_node,0);
%extract intrinsic camera parameters
width = wb_camera_get_width(camera);
height = wb_camera_get_height(camera);
fov = wb_camera_get_fov(camera)
f = width/(2 * tan(fov/2)) %focal length in pixels

%define principle point, bearing in mind that in matlab top left corner is (1,1)
%remember to draw this and check.
u0 = (width+ 1)/2 ;
v0 = (height+1)/2 ;
origin = [1,1];
centre = origin + [u0,v0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DESIRED POINT SET UP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set up P_des, Order is Top Left, TR, BL,  BR;
Length = 44;
Floor_offset = 8
%P_des = [[u0 - Length/2, v0 - Length/2]; %TL
%         [u0 + Length/2, v0 - Length/2]; %TR
%         [u0 - Length/2, v0 + Length/2 - Floor_offset]; %BL
%         [u0 + Length/2, v0 + Length/2 - Floor_offset]] %BR

P_des = [[-15.6628, -19.6319];
        [15.7104, -19.6371];
        [-15.6604, 9.7918];
        [15.7218, 9.7852]];
P_des = P_des + [u0,v0];
%normalise p_des for the controller.
%P_des_norm = (P_des - [u0,v0])'; %Transposed for array arrangement in next line
P_des_norm = (P_des - [u0,v0])
P_des_norm = P_des_norm';
%Now, make this a single row.
P_des_norm = P_des_norm(:) % Here all the different coloumns are stacked into 1,

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SEARCH REGION FOR CORNER DETECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%set up search region for corner detection. Whole image for now.
% ROI_W = 20;
% ROI_H = 20;
% ROI = [origin;
       % origin + [0, height];
       % origin + [width, 0];
       % origin + [width, height]];
% ROI = [(u0 - ROI_W), (v0 - ROI_H), ROI_W, ROI_H];

%SearchArea = [u0-20,v0 -20,20,20];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MOTOR SET UP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%set up motors
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
%Set motor speeds
wb_motor_set_position(left_motor,inf);
wb_motor_set_position(right_motor,inf);
wb_motor_set_velocity(left_motor,0.0);
wb_motor_set_velocity(right_motor,0.0);




%Set up block Z position Tracking
block_node = wb_supervisor_node_get_from_def('target');
bot_node = wb_supervisor_node_get_from_def('bot')
% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination

%initial capture
wb_robot_step(TIME_STEP);
%find intial Square points
rgb = wb_camera_get_image(camera);
I = rgb2gray(rgb);
if plot_img == ON
  imshow(I); hold on;
  points = detectHarrisFeatures(I);
  strong = points.selectStrongest(4);
  plot(P_des(:,1),P_des(:,2),'ro');
  plot(strong);
  drawnow;
end
%intialise point tracking
B = sortrows(strong.Location,1);
Left = B(1:2,:);
Right = B(3:4,:);

C = sortrows(Left,2);
TL = C(1,:);
BL = C(2,:);

D = sortrows(Right,2);
TR = D(1,:);
BR = D(2,:);

p_init = [TL;TR;BL;BR]


p_prior = p_init;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DATA FOR PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%init data variables
phiLplot = [];
phiRplot = [];
phiDiff = [];
t = [];
e_data = [];
norm_e_data = [];
p_data = [];
Jp_data = [];
%Set up figures for diff data
cam = figure(1);
%harris = figure(5);

time = 0;
%while wb_robot_step(TIME_STEP) ~= -1
loss_count = 0;
loss_data = [];
while time <= sim_duration;
  %progress the simulation by one time step.
  wb_robot_step(TIME_STEP); 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CORNER DETECTION + IMAGE CAPTURE
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % read the sensors, e.g.:
  rgb = wb_camera_get_image(camera);
  I = rgb2gray(rgb);

  % Process here sensor data, images, etc.
  corners = detectHarrisFeatures(I);
  strong = corners.selectStrongest(4);
  p = strong.Location;

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CORNER SORTING + CENTRING
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  %strong.Count
  if strong.Count == 4
  
    norms = nan(size(p_prior,1),size(p,1));
    %Calculate euclidean distance between points (i.e. the cost matrix)
    for i = 1:size(p_prior,1)
      for j = 1:size(p,1)
        norms(i,j) = norm(p_prior(i,:) - p(j,:));
      end
    end
    
    %Put into Hungarian Method
    [M,uR,uC] = matchpairs(norms,costUnmatched);
    
    %sort the M matrix
    M = sortrows(M,1);
    if size(M,1) == 4
      %assign the points
      p_sort = nan(size(p,1),2);
      for i = 1:size(M,1)
        p_sort(M(i,1),:) = p(M(i,2),:);
      end
      
      %store p to p_prior
      p_prior = p_sort;
      %centre p
      p_cent = p_sort - [u0,v0]
      
     
     % make points relative to the centre. The controller is derived like this
     u1 = p_cent(1,1) ;
     v1 = p_cent(1,2) ;
     u2 = p_cent(2,1) ;
     v2 = p_cent(2,2) ;
     u3 = p_cent(3,1) ;
     v3 = p_cent(3,2) ;
     u4 = p_cent(4,1) ;
     v4 = p_cent(4,2) ;
     
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % JACOBIAN
     % The f passed into the jacobian is already in pixels
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     %Just pick a number for now. Will return to funky forms later.
     %Might just pass blocks z co-ord directly instead and assume lidar gave it to me.
     %block_translation = wb_supervisor_node_get_position(block_node);
     %bot_translation = wb_supervisor_node_get_position(bot_node);
     %Z = abs(block_translation(3) - bot_translation(3)) %m
     Z = 1;
     Jp1 = [[u1/Z, -(f^2 + u1^2)/f];
            [v1/Z,   -u1*v1/f]];
     Jp2 = [[u2/Z, -(f^2 + u2^2)/f];
            [v2/Z,   -u2*v2/f]];
     Jp3 = [[u3/Z, -(f^2 + u3^2)/f];
            [v3/Z,   -u3*v3/f]];
     Jp4 = [[u4/Z, -(f^2 + u4^2)/f];
            [v4/Z,   -u4*v4/f]];
     
     %Stack 'em
     Jp = [Jp1;Jp2;Jp3;Jp4];
     Jp_cond = cond(Jp);
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %   IMPLEMENT IVBS CONTROLLER%
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     
     
     %process p
     p_T = p_cent';
     p = p_T(:);
     e = (P_des_norm - p);
     norm_e = norm(e);
     
     %chosse k = 2
     %k = [0.2 0;0.1];
     k = gain;
     v = k * pinv(Jp) * e;
     
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % SATURATION CHECK AND LIMITER %
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % saturation check only on v not psi.
     v_lim = a-a/b*abs(v(2));
     if( abs(v(1)) > v_lim )
       v(1) = sign(v(1))*v_lim;
     end
     
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %        KINEMATIC MAPPING          %
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     %Don't need this for now, as v can be passed on as is.
     v_x = v(1);
     omega_z = -v(2);    %Polarity corrected due to mapping from camera frame to body frame
     vel = [v_x;omega_z];
     
     %Standard Kinematic Mapping
     %map = [[1 -d/2];
     %      [1 d/2]]; 
     
     % Kmap with slip
     if slip_toggle == ON
       map = [[1/sL_calc -d/(2*(1/sL_calc))];
              [1/sR_calc d/(2*(1/sR_calc))]]; 
     else %use standard kinematic mapping
       map = [[1 -d/2];
              [1 d/2]];
     end;
     
     phi = 1/r * map * vel;
        
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %             DEAD BAND                %
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     if (norm_e > d_bnd_tol) || (d_bnd_toggle == OFF)
       phiL = phi(1);
       phiR = phi(2);
     else
       phiL = 0;
       phiR = 0;
     end
     % disp("v:");
     % disp(v);
     % disp("phi:");
     % disp(phi);
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % MOVE THE ROBOT                    %
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     %Actuate motors, this just feeds the motors the phi values.
     wb_motor_set_velocity(left_motor,phiL);
     wb_motor_set_velocity(right_motor,phiR); 
    else   
      plot_points
    end
    %This is just crash prevention at the moment...  
  elseif (strong.Count < 4) && (strong.Count ~= 1)
    if loss_count == 0
      first_loss = wb_robot_get_time();
    end
    loss_count = loss_count + 1;
    
    disp("insufficient data");
    e = nan(8,1);
    norm_e = NaN;
    phiL = NaN;
    phiL = NaN;
    Jp_cond = NaN;
  elseif strong.Count == 1 
    if loss_count == 0
      first_loss = wb_robot_get_time();
    end
    loss_count = loss_count + 1;
    disp("insufficient data");
    e = nan(8,1);
    norm_e = NaN;
    phiL = NaN;
    phiL = NaN;
    Jp_cond = NaN;
  end 
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%
  %       Store data       %
  %%%%%%%%%%%%%%%%%%%%%%%%%%
  
  e_data = [e_data,e];
  norm_e_data = [norm_e_data,norm_e];
  Jp_data = [Jp_data, cond(Jp)];
  phiLplot = [phiLplot,phiL];
  phiRplot = [phiRplot,phiL];
  phiDiff = [phiDiff,(phiL - phiR)];
  t = [t,wb_robot_get_time()];
  p_data(:,:,end + 1) =  p_sort;
  loss_data = [loss_data, loss_count]; 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  % PLOT IMAGE
  %%%%%%%%%%%%%%%%%%%%%%%%%%%

  if (plot_img == ON)
    figure(cam);
    imshow(I); hold on;
    plot(P_des(:,1),P_des(:,2),'ro');
    plot(strong);
  
  % figure(harris);
  % hold on;
  % plot(strong);
  end
  
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

  %%%%%%%%%%%%%%%%%%%%%%%%%
  %     UPDATE TIME
  %%%%%%%%%%%%%%%%%%%%%%%%%
  time = wb_robot_get_time();
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT POINTS,IMAGE AND DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (plot_error == ON)
  error = figure(2);
  figure(error);
  clf;
  plot(t,e_data,'-','LineWidth',1);
  legend('Point 1 x','Point 1 y', 'Point 2 x', 'Point 2 y', 'Point 3 x', 'Point 3 y', 'Point 4 x', 'Point 4 y');
  
  title('Error vs Time');
  ylabel('error (pixels)');
  xlabel('t (seconds)');
  if needed
    for i = 1:4
      if dir_changes == 4
        corner_name = sprintf("Corner %g", i)
        xline(i*corner_time,'--r',corner_name);
      elseif (mod(i,dir_changes) == 0)
        corner_name = sprintf("change %g", i/2);
        xline(i*corner_time,'--r',corner_name);
      end
    end
    
    if(loss_count ~= 0)
      loss_time = sprintf("Point Lost at %g s", first_loss);
      xline(first_loss,'--m',loss_time);
    end
  end
  grid on;
  sample_name = sprintf("Speed_%g_mps", Block_speed);
  file_name = file_path + "error_" + sample_name + ".png";
  exportgraphics(error,file_name);
end  

if (norm_e_plot == ON)
  norm_error = figure(3);
  figure(norm_error);
  clf;
  plot(t,norm_e_data,'c-','LineWidth',1);
  title('Error Norm vs Time');
  ylabel('error norm (pixels)');
  xlabel('t (seconds)');
  if needed
    for i = 1:4
      if dir_changes == 4
        corner_name = sprintf("Corner %g", i)
        xline(i*corner_time,'--r',corner_name);
      elseif (mod(i,dir_changes) == 0)
        corner_name = sprintf("change %g", i/2);
        xline(i*corner_time,'--r',corner_name);
      end
    end
    
    if(loss_count ~= 0)
      loss_time = sprintf("Point Lost at %g s", first_loss);
      xline(first_loss,'--m',loss_time);
    end
  end
  grid on;

  sample_name = sprintf("Speed_%g_mps", Block_speed);
  file_name = file_path + "norm_error_" + sample_name + ".png";
  exportgraphics(norm_error,file_name);
end 

if (plot_Jp == ON)
  cond_num = figure(4);
  figure(cond_num);
  clf;
  plot(t,Jp_data,'c-','LineWidth',1);
  title('Image Jacobian Condition Number vs Time');
  ylabel('Condition Number');
  xlabel('t (seconds)');
  if needed
    for i = 1:4
      if dir_changes == 4
        corner_name = sprintf("Corner %g", i)
        xline(i*corner_time,'--r',corner_name);
      elseif (mod(i,dir_changes) == 0)
        corner_name = sprintf("change %g", i/2);
        xline(i*corner_time,'--r',corner_name);
      end
    end
    
    if(loss_count ~= 0)
      loss_time = sprintf("Point Lost at %g s", first_loss);
      xline(first_loss,'--m',loss_time);
    end
  end
  grid on;

  sample_name = sprintf("Speed_%g_mps", Block_speed);
  file_name = file_path + "cond_num_" + sample_name + ".png";
  exportgraphics(cond_num,file_name);
end  

if (plot_wheel_speeds == ON)
  wheels = figure(5);
  figure(wheels);
  clf;
  plot(t,phiLplot,'c','LineWidth',1);
  hold on;
  plot(t,phiRplot,'m','LineWidth',1);
  plot(t,phiDiff,'y','LineWidth',1);
  

  title('Wheel Speeds vs Time');
  ylabel('Wheel Speed (rad/s)');
  xlabel('t (seconds)');
  if needed
    for i = 1:4
      if dir_changes == 4
        corner_name = sprintf("Corner %g", i)
        xline(i*corner_time,'--r',corner_name);
      elseif (mod(i,dir_changes) == 0)
        corner_name = sprintf("change %g", i/2);
        xline(i*corner_time,'--r',corner_name);
      end
    end
    
    if(loss_count ~= 0)
      loss_time = sprintf("Point Lost at %g s", first_loss);
      xline(first_loss,'--m',loss_time);
    end
  end
  grid on;
  legend('Left Wheel Speed','Right Wheel Speed', 'Wheel Speed Difference');
  sample_name = sprintf("Speed_%g_mps", Block_speed);
  file_name = file_path + "wheels_" + sample_name + ".png";
  exportgraphics(wheels,file_name);
end 

if (plot_points == ON)
  points = figure(6);

  figure(points); hold on;
  clf;
  
  
  for i = 1:4
    u = reshape(p_data(i,1,:), [], 1);
    v = reshape(p_data(i,2,:), [], 1);
    
    plot(u,v,'-o')
    hold on;
  end
  
  % for i = 1:4
    % plot(squeeze(p_data(i,1,:)), ...
         % squeeze(p_data(i,2,:)), '-o');
  % end

  title('Corner path movement in discrete image plane');
  xlabel('u (pixels)');
  ylabel('v (pixels)');
  axis ij;
  grid on;
  axis equal;
  legend('Top Left Corner','Top Right Corner', 'Bottom Left Corner', 'Bottom Right Corner');
  sample_name = sprintf("Speed_%g_mps", Block_speed);
  file_name = file_path + "points_" + sample_name + ".png";
  exportgraphics(points,file_name);
end 

if plot_loss == ON
  loss = figure(7);
  figure(cond_num);
  clf;
  plot(t,loss_data,'c-','LineWidth',1);
  title('Point loss instances vs Time');
  ylabel('number of point loss instances');
  xlabel('t (seconds)');
  if needed
    for i = 1:4
      if dir_changes == 4
        corner_name = sprintf("Corner %g", i)
        xline(i*corner_time,'--r',corner_name);
      elseif (mod(i,dir_changes) == 0)
        corner_name = sprintf("change %g", i/2);
        xline(i*corner_time,'--r',corner_name);
      end
    end
  end
  grid on;

  sample_name = sprintf("Speed_%g_mps", Block_speed);
  file_name = file_path + "point_loss" + sample_name + ".png";
  exportgraphics(cond_num,file_name);
end

% Calculate statistics, 4 and 6 determined from graphs for square and approach. t >= 4& t <= 6
% Use t >= (max(t)/2) & t <= max(t)
idx = t >= (max(t)/2) & t <= max(t);

mean_error = mean(norm_e_data(idx), 'omitnan');
std_error  = std(norm_e_data(idx), 'omitnan');

mean_condition = mean(Jp_data(idx), 'omitnan');
std_condition  = std(Jp_data(idx), 'omitnan');

% Create results table
Results = table( ...
    mean_error, std_error, ...
    mean_condition, std_condition, ...
    'VariableNames', { ...
    'MeanError', 'StdError', ...
    'MeanConditionNumber', 'StdConditionNumber'});
    

file_name = file_path + sample_name + 'IBVS_statistics.csv';
% Export to CSV
writetable(Results,file_name);