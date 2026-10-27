% MATLAB controller for Webots
% File:          trajecotry.m
% Date:
% Description:
% Author:
% Modifications:
function trajecotry

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 32;

  d = 0.052; %m
  r = 0.0205; %m
  phi_lim = 6.279;
  a = r*phi_lim;
  b = 2*r/d*phi_lim;
  
  dT = TIME_STEP/1000; %update/sample time
  %dT = TIME_STEP;
  timer = rateControl(1/dT);
  k_psi = 0.5; %orientation controller proportional gain
  k_v1 = 2; %linear velocity proportional gain
  k_v2 = 2; %linear velocity integral gain
  d_star = 0.12; %look ahead distance
  L = 0.5;
  s1 = [linspace(d_star,L,1e3); zeros(1,1e3)];
  s2 = [L*ones(1,1e3); linspace(0,L,1e3)];
  s3 = [linspace(L,0,1e3); L*ones(1,1e3)];
  s4 = [zeros(1,1e3); linspace(L,0,1e3)];
  S = [s1 s2 s3 s4];
  ev_int = 0;
  t = 0;
  psi_des_past = 0;
  pTrail = [];
 %noOfLoops = 2;
  S_ = [];
  S_ = S;
  T = eye(3);
  psi = 0;
  
  bot_node = wb_supervisor_node_get_from_def('bot')
  bot_translation = wb_supervisor_node_get_position(bot_node);
  tvec = bot_translation' %m
  %tvec = zeros(3,1);
  
% get and enable devices, e.g.:
  camera = wb_robot_get_device('camera');
  wb_camera_enable(camera, TIME_STEP);



  %Init Actuators
  left_motor = wb_robot_get_device('left wheel motor');
  right_motor = wb_robot_get_device('right wheel motor');
  %Set motor speeds
  wb_motor_set_position(left_motor,inf);
  wb_motor_set_position(right_motor,inf);
  wb_motor_set_velocity(left_motor,0.0);
  wb_motor_set_velocity(right_motor,0.0);

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
  i = 1;
while i <= size(S_,2)
   wb_robot_step(TIME_STEP)
   %figure(1),clf,hold on
   %plot(S(1,:),S(2,:),'k--')
   p_des = S_(:,i);
   e_p_wf = p_des(1:2)-tvec(1:2);
   %orientation control
   % psi = robotPose(3);
   psi_des = atan2(e_p_wf(2),e_p_wf(1));
  
   e_psi = psi_des-psi;
   if( abs(e_psi)>pi )
   e_psi = -sign(e_psi)*( 2*pi-abs(e_psi) );
   end
   u_psi = k_psi*e_psi;
   %position control
   ev = norm(e_p_wf)-d_star;
   ev_int = ev_int+dT*ev;
  
   u_v = k_v1*ev+k_v2*ev_int;
   
   %saturation detector
   v_lim = a-a/b*abs(u_psi);
   if( abs(u_v) > v_lim )
      u_v = sign(u_v)*v_lim;
   end
   
   phi_des = [1/r -d/(2*r);1/r d/(2*r)] *[u_v,u_psi]';

   phiL = phi_des(1);
   phiR = phi_des(2);
   v = r*( phiL+phiR )/2;
   psi_dot = r*(phiR-phiL)/d;
  
   %send feedback to motors
   wb_motor_set_velocity(left_motor,phiL);
   wb_motor_set_velocity(right_motor,phiR); 
   
   R = [cos(psi_dot*dT) -sin(psi_dot*dT);
   sin(psi_dot*dT) cos(psi_dot*dT)];
   if( psi_dot==0 )
   J = eye(2,2);
   else
   J = [sin(psi_dot*dT)/(psi_dot*dT) (cos(psi_dot*dT)-1)/(psi_dot*dT);
   (1-cos(psi_dot*dT))/(psi_dot*dT) sin(psi_dot*dT)/(psi_dot*dT) ];
   end
   T_inc = [R J*[v 0]'*dT; 0 0 1];
   T = T*T_inc;
   %bot_translation = wb_supervisor_node_get_position(bot_node);
   %tvec = bot_translation %m
   tvec = [T(1:2,3);1];
   psi = atan2(T(2,1),T(1,1));
   q = [cos(psi/2) 0 0 sin(psi/2)];
   pTrail = [pTrail tvec(1:2)];
   %plot(pTrail(1,:),pTrail(2,:),'b-',lineWidth=2)
   %axis([-1 1 -1 1]),grid on
   %plotTransforms(tvec', q, "MeshFilePath", "groundvehicle.stl", "Parent", gca,"View","2D", "FrameSize", d/.5);
   %plot(p_des(1),p_des(2),'ro')
   t = t+dT;
   %title(['Time: ',num2str( t ), ' seconds'])
   %text(0,18,{['Orientation error: ',num2str( e_psi*180/pi,'%.2f' ), ' deg a'], ...
   %['Look ahead distance: ',num2str( norm(e_p_wf),'%.2f' ), ' m']})
   %drawnow
   waitfor(timer);
  % if i == (length(S_))
    % i = 1;
  % else
    % if i + 5 > size(S_,2)
      % i = i + 1;  
    % else
      % i = i + 5;
    % end
  % end
  i = i + 5;
  % disp("final time is");
  % disp(t);  
end

% cleanup code goes here: write data to files, etc.
