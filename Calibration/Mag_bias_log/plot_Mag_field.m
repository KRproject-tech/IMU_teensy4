clc
clear all
close all hidden

%% delete
delete( '*.asv')

%% parameters

file_name = 'Mag_bias_log.txt';


%% load

data = importdata( file_name);

time_m = data(:,1);
acc_m = data(:,2:4);
mag_m = data(:,5:end);

%%[*] sphere fitting
% The output 
% R = radius
% rc_x = X coordinate of the center
% rc_y = Y coordinate of the center
% rc_z = Z coordinate of teh center
[ R, rc_x, rc_y, rc_z] = sphereFit( mag_m);
rc_vec = [  rc_x;
            rc_y;
            rc_z];

[ X, Y, Z] = sphere( 100);
X = R*X + rc_x;
Y = R*Y + rc_y;
Z = R*Z + rc_z;



%% plot


i_ax = 1;

h_fig(1) = figure(1);
set( h_fig(1), 'Position', [100 100 600 600])
h_ax(i_ax) = axes( 'Parent', h_fig(1), 'FontSize', 15);


plot3( h_ax(i_ax), mag_m(:,1),  mag_m(:,2),  mag_m(:,3), 'bo', 'LineWidth', 2);
hold( h_ax(i_ax), 'on')
surf( h_ax(i_ax), X, Y, Z, 'FaceAlpha', 0.5)
view( h_ax(i_ax), [ 1 1 0.5])
xlabel( h_ax(i_ax), '{\itm_x} [-]', 'FontName', 'Times New Roman')
ylabel( h_ax(i_ax), '{\itm_y} [-]', 'FontName', 'Times New Roman')
zlabel( h_ax(i_ax), '{\itm_z} [-]', 'FontName', 'Times New Roman')
grid( h_ax(i_ax), 'on')
axis( h_ax(i_ax), 'equal')


set( h_ax(i_ax), 'FontName', 'Times New Roman')
