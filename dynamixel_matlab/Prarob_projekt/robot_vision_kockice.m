
% grab an image 
camera_info = imaqhwinfo('winvideo');
id = camera_info.DeviceInfo.DeviceID
format = camera_info.DeviceInfo.DefaultFormat
video_obj = videoinput('winvideo', id, format);
video_obj.ReturnedColorSp
ace = 'rgb';
f = figure('Visible', 'off'); 
vidRes = video_obj.VideoResolution;
imageRes = fliplr(vidRes);
hImage = imshow(zeros(imageRes));
% preview video object
preview(video_obj, hImage);
waitforbuttonpress;
stoppreview(video_obj);
% start to enable grab
start(video_obj);
img = getdata(video_obj);
start(video_obj);
img = getdata(video_obj);
img = img(:, :, (1:3));
stop(video_obj);
clear video_obj
imwrite(img,'slika_projekt.jpg')

%% 
img = imread('slika_projekt.jpg');

[width, height, numberOfColorChannels] = size(img);
figure(1)
imshow(img) 
hold on
plot(height/2,width/2, 'r+', 'MarkerSize', 30, 'LineWidth', 2)
hold off
% x i y su obrnuti zato sto su osi kod robota drukčije
%% BLACK
x_black_1 = center_2_x
y_black_1 = center_2_y
x_black_2 = center_1_x
y_black_2 = center_1_y
x_black_3 = center_3_x
y_black_3 = center_3_y
%%
black_location_1 = find_object_black(x_black_1,y_black_1);
x_bla_1 = black_location_1(2)/1000
y_bla_1 = black_location_1(1)/1000
black_location_2 = find_object_black(x_black_2,y_black_2);
x_bla_2 = black_location_2(2)/1000
y_bla_2 = black_location_2(1)/1000
black_location_3 = find_object_black(x_black_3,y_black_3);
x_bla_3 = black_location_3(2)/1000
y_bla_3 = black_location_3(1)/1000

%%
izravnaj();
%% GREEN
u_g = x_green
v_g = y_green
green_location = find_object(u_g,v_g);
x_g = green_location(2)/1000
y_g = green_location(1)/1000

%% send data green

inverzna_kinematika(x_g,y_g);
%%
inverzna_kinematika(x_bla_1,y_bla_1);
%% BLUE
u_b = x_blue
v_b = y_blue
blue_location = find_object(u_b,v_b);
x_blu = blue_location(2)/1000
y_blu = blue_location(1)/1000
%% send data blue
inverzna_kinematika(x_blu,y_blu);
%%
inverzna_kinematika(x_bla_2,y_bla_2);
%% YELLOW
u_y = x_yellow
v_y = y_yellow
yellow_location = find_object(u_y,v_y);
x_y = yellow_location(2)/1000
y_y = yellow_location(1) /1000
%% send data yellow
inverzna_kinematika(x_y,y_y);
%%
inverzna_kinematika(x_bla_3,y_bla_3);
%%
function centroid_world = find_object(u,v)
cx = 512;
cy = 288;
fx = 1154.087;
fy = 1114.444;

z = 590;
x = (z/fx)*(u - cx);
y = (z/fy)*(v - cy);

x_camera = [x];
y_camera = [y];
z_camera = z;

% convert to homogeneous coordinates
centroid_camera_h = [x_camera; y_camera; z_camera; 1];

% calculate centroid in world coordinate frame
orientation_from_world = [1 0 0; 0 -1 0; 0 0 -1];
deltaY = 130 - 10;
location_from_world = [-10 120 590];
T_world_camera = [orientation_from_world, location_from_world'; 0 0 0 1];

centroid_world_h = T_world_camera * centroid_camera_h;
centroid_world = centroid_world_h(1:3,1);
%%
end
%%-----------------------------------------------------------------------
function centroid_world = find_object_black(u,v)
cx = 512;
cy = 288;
fx = 1154.087;
fy = 1114.444;

z = 590;
x = (z/fx)*(u - cx);
y = (z/fy)*(v - cy);

x_camera = [x];
y_camera = [y];
z_camera = [z];

% convert to homogeneous coordinates
centroid_camera_h = [x_camera; y_camera; z_camera; 1];

% calculate centroid in world coordinate frame
orientation_from_world = [1 0 0; 0 -1 0; 0 0 -1];
deltaY = 130 - 10;
location_from_world = [35 110 590];
T_world_camera = [orientation_from_world, location_from_world'; 0 0 0 1];

centroid_world_h = T_world_camera * centroid_camera_h;
centroid_world = centroid_world_h(1:3,1);
end
%% inv
function results = inverzna_kinematika(x, y);
            z=0.045;
            y1=y;
            y=-y1;
            a1=0.09;
            a2=0.19;
            a3=0.18;
 
            q1 = atan2(y, x);
            r1 = sqrt(x^2+y^2);
            r2 = z - a1;
            fi2 = atan2(r2, r1);
            r3 = sqrt(r2^2+r1^2);
            fi1 = acos((a3^2 - a2^2 - r3^2)/(-2*a2*r3));
            q2 = pi/2 - (fi1+fi2);
            fi3 = acos((r3^2-a2^2 -a3^2)/(-2*a2*a3));
            q3=pi-fi3;
 
 
            p = [angle(q1), angle(q2), angle(q3)]
            pd = [rad2deg(q1), rad2deg(q2), rad2deg(q3)]
            
         
 
            set_param('simulation_three_motors/motor1_pose', 'Value', mat2str(round(pd(1),2)));
            set_param('simulation_three_motors/motor2_pose', 'Value', mat2str(round(pd(2),2)));
            set_param('simulation_three_motors/motor3_pose', 'Value', mat2str(round(pd(3),2)));
end
        
function izravnaj 
     pd1 = 0
     pd2 = 0
     pd3 = 90
     set_param('simulation_three_motors/motor1_pose', 'Value', mat2str(pd1));
            set_param('simulation_three_motors/motor2_pose', 'Value', mat2str(pd2));
            set_param('simulation_three_motors/motor3_pose', 'Value', mat2str(pd3));
end
