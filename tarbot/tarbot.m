% MATLAB controller for Webots
% File:          tarbot.m
% Date:
% Description:
% Author:
% Modifications:
function tarbot

file_path = "C:\Users\Kyle\Documents\Uni\Final year\FYP\Data\Plots\lum 0\Approach\gain 1\Time steps\";

%sim_duration = perimeter/v;
sim_duration = 51.2;
Approach = 1;
if Approach
  sim_duration = 10;
end
time = 0;

%%%%%%%%%%%%%%$
% SIM TOGGLES %
%%%%%%%%%%%%%%%
ON = 1;
OFF = 0;

Position_plot = ON;
Straightline = OFF;
Block_speed = 0.2;

v = Block_speed;
TIME_STEP = 32;
dT = TIME_STEP/1000;

block_node = wb_supervisor_node_get_from_def('target');
bot_node = wb_supervisor_node_get_from_def('bot');
tarbot_node = wb_supervisor_node_get_from_def('tarbot');

change = 0;

x_sum = 0;
y_sum = 0;
length = 1;
i = 0;
if Straightline
  perimeter = length * 2;
else
  perimeter = length * 4;
end
block_pos_data = [];
bot_pos_data = [];





while time <= sim_duration;
  %progress the simulation by one time step.
  wb_robot_step(TIME_STEP); 
  
  translation_field = wb_supervisor_node_get_field(block_node, 'translation');
  rotation_field = wb_supervisor_node_get_field(block_node, 'rotation');
  block_pos = wb_supervisor_node_get_position(block_node);
  bot_pos = wb_supervisor_node_get_position(bot_node);
  tarbot_pos = wb_supervisor_node_get_position(tarbot_node);
  tarbot_rot = wb_supervisor_node_get_field(tarbot_node, 'rotation');
  tarbot_rot = wb_supervisor_field_get_sf_rotation(tarbot_rot);
  %disp(tarbot_rot);
  new_field = tarbot_pos + [0,0,0.075];
  block_translation = wb_supervisor_node_get_position(block_node);

   
  
  wb_supervisor_field_set_sf_vec3f(translation_field, new_field);
  wb_supervisor_field_set_sf_rotation(rotation_field, tarbot_rot);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  % STORE DATA
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  block_pos_data = [block_pos_data;[block_pos(1),block_pos(2)]];
  bot_pos_data = [bot_pos_data;[bot_pos(1),bot_pos(2)]];
  
  %%%%%%%%%%%%%%%%%%%%%%%%%
  %     UPDATE TIME
  %%%%%%%%%%%%%%%%%%%%%%%%%
  time = wb_robot_get_time();
  %disp(time);
end

if (Position_plot == ON)
  
  position = figure(1);
  figure(position);
  clf;
  plot(bot_pos_data(:,1),bot_pos_data(:,2),'-o','LineWidth',1);
  hold on;
  plot(block_pos_data(:,1),block_pos_data(:,2),'-o','LineWidth',1);
  
  sample_name = sprintf("Speed_%g_mps", v);
  file_name = file_path + "position_" + sample_name + ".png";
  title('Robot and block trajectories',sample_name);
  ylabel('y (m)');
  xlabel('x (m)');
  grid on;
  legend('bot', 'block');
  exportgraphics(position,file_name)
end 