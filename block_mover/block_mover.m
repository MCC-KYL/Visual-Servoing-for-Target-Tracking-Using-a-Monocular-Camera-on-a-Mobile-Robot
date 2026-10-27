% MATLAB controller for Webots
% File:          block_mover.m
% Date:
% Description:
% Author:
% Modifications:
function block_mover

file_path = "C:\Users\Kyle\Documents\Uni\Final year\FYP\Data\Plots\lum 3\Line\realisms\slip\";
%%%%%%%%%%%%%%$
% SIM TOGGLES %
%%%%%%%%%%%%%%%
ON = 1;
OFF = 0;

Position_plot = ON;
Straightline = ON;
Block_speed = 0.08;

v = Block_speed;
TIME_STEP = 64;
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



sim_duration = perimeter/v;
time = 0;

while time <= sim_duration;
  %progress the simulation by one time step.
  wb_robot_step(TIME_STEP); 
  
  translation_field = wb_supervisor_node_get_field(block_node, 'translation');
  block_pos = wb_supervisor_node_get_position(block_node);
  bot_pos = wb_supervisor_node_get_position(bot_node);
  disp(translation_field);
  block_translation = wb_supervisor_node_get_position(block_node);
  x0 = block_translation(1);
  y0 = block_translation(2);
  
  
  if (x_sum <= length && change == 0)
    %forward
    if (x0 + v*dT) >= length
      x = length;
      x_sum = length;
    else
      x = x0 + v*dT;
      x_sum = x_sum + v*dT;
    end
    
    new_value = [x, y0, 0.075];
    if x_sum >= length
      if Straightline
        change = 2;
      else
        change = 1;
      end
    end
  elseif (y_sum <= length && change == 1 && (Straightline == OFF))
    %left
    
    if (y0 + v*dT) >= length
      y = length;
      y_sum = length;
    else
      y = y0 +v*dT;
      y_sum = y_sum + v*dT;
    end

    new_value = [x0, y, 0.075];
    if y_sum >= length
      change = 2;
    end
    
  elseif (x_sum >= 0 && change == 2)
    %backward
    if (x0 - v*dT) <= 0
      x = 0;
      x_sum = 0;
    else
      x = x0 - v*dT;
      x_sum = x_sum - v*dT;
    end
    new_value = [x, y0, 0.075];
    if x_sum <= 0
      if Straightline
        change = 0;
      else
        change = 3;
      end
    end
    
    
  elseif (y_sum >= 0 && change == 3 && (Straightline == OFF))
    %right
    
    if (y0 - v*dT) <= 0
      y = 0;
      y_sum = 0;
    else
      y = y0 - v*dT;
      y_sum = y_sum - v*dT;
    end
    
    new_value = [x0, y, 0.075];
    if y_sum <= 0
      change = 0;
    end
  end

  wb_supervisor_field_set_sf_vec3f(translation_field, new_value);
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  % STORE DATA
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  block_pos_data = [block_pos_data;[block_pos(1),block_pos(2)]];
  bot_pos_data = [bot_pos_data;[bot_pos(1),bot_pos(2)]];
  
  %%%%%%%%%%%%%%%%%%%%%%%%%
  %     UPDATE TIME
  %%%%%%%%%%%%%%%%%%%%%%%%%
  time = wb_robot_get_time();
  
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
