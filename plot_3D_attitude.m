clc
clear all
close all hidden

%% delete
delete('*.asv')

%% parameter

s = serial('COM3');
fopen( s);


Rx = 2.0;                                                                               %% x軸切片 [m]
Ry = 1.0;                                                                               %% y軸切片 [m]
Rz = 0.5;                                                                               %% z軸切片 [m]


tild_mat = @( x)[   0           -x(3)       x(2)    ;
                    x(3)        0           -x(1)	;
                    -x(2)       x(1)    	0       ];

L_mat = @( eps_vec)( [ -eps_vec(2:4) eps_vec(1)*eye(3)-tild_mat( eps_vec(2:4))] ); 
E_mat = @( eps_vec)( [ -eps_vec(2:4) eps_vec(1)*eye(3)+tild_mat( eps_vec(2:4))] ); 

A_mat = @( eps_vec)( E_mat( eps_vec)*L_mat(eps_vec).' );                                %% 回転基本行列


%% 局所座標系データ

[ x_body, y_body, z_body] = ellipsoid( 0, 0, 0, Rx, Ry, Rz, 20);
r_body_T = [ x_body(:) y_body(:) z_body(:)];

R_G_vec_T = @( eps_vec)( r_body_T*A_mat( eps_vec).' );




%% plot

i_ax = 1;

h_fig(1) = figure(1);
set( h_fig(1), 'Position', [100 100 600 600], 'render', 'zbuffer')
h_ax(i_ax) = axes( 'Parent', h_fig(1), 'FontSize', 15);




eps_vec = [ 1 0 0 0].';

R_G_vec_T_v = R_G_vec_T( eps_vec);

X_G = reshape( R_G_vec_T_v(:,1), size( x_body, 1), []);
Y_G = reshape( R_G_vec_T_v(:,2), size( y_body, 1), []);
Z_G = reshape( R_G_vec_T_v(:,3), size( z_body, 1), []);


h_plot(1) = surf( h_ax(i_ax), X_G, Y_G, Z_G);
hold( h_ax(i_ax), 'on')
plot3( h_ax(i_ax), [-1 1]*3, [0 0], [0 0])
plot3( h_ax(i_ax), [0 0], [-1 1]*3, [0 0])
plot3( h_ax(i_ax), [0 0], [0 0], [-1 1]*2)
% light
view( h_ax(i_ax), [ 1 1 0.5])
xlabel( h_ax(i_ax), '{\itX} [m]')
ylabel( h_ax(i_ax), '{\itY} [m]')
zlabel( h_ax(i_ax), '{\itZ} [m]')
% box( h_ax(i_ax), 'on')

axis equal
% set( h_ax(i_ax), 'FontName', 'Times New Roman')


h_txt(1) = text( -1, -1, 1, [ 'Time = ', num2str( 0, '%0.2f'), ' [s]'],...
                'FontSize', 12, 'FontName', 'Times New Roman', 'BackgroundColor', 'g');
            
            

ButtonHandle = uicontrol( h_fig(1), 'Style', 'PushButton', ...
                                    'String', 'Stop loop', ...
                                    'Callback', 'delete(gcbf)');            
            

 i_ax = i_ax + 1;                               
                                
 
 

h_fig(2) = figure(2);
set( h_fig(2), 'Position', [700 100 600 600], 'render', 'zbuffer')
h_ax(i_ax) = axes( 'Parent', h_fig(2), 'FontSize', 15);
                     
h_plot(2) = quiver3( h_ax(i_ax), 0, 0, 0, 1, 0, 0);
view( h_ax(i_ax), [ 0 0 1])
xlabel( h_ax(i_ax), '{\itm_x} [-]')
ylabel( h_ax(i_ax), '{\itm_y} [-]')
zlabel( h_ax(i_ax), '{\itm_z} [-]')




%% animation                     
while 1   
    

	data = fscanf( s);

    
    %% "time q0 q1 q2 q3"
    num_data = str2num( data);
    
    if isempty( num_data) ~= 1 && length( num_data) == 8
        
        time = num_data(1); 
        eps_vec = num_data(2:5).';
        mag_vec = num_data(6:end).';
        
        
        if mod( round( time*1000), 10) == 0
            
            R_G_vec_T_v = R_G_vec_T( eps_vec);
            X_G = reshape( R_G_vec_T_v(:,1), size( x_body, 1), []);
            Y_G = reshape( R_G_vec_T_v(:,2), size( y_body, 1), []);
            Z_G = reshape( R_G_vec_T_v(:,3), size( z_body, 1), []);

            %% plot
            %%[*] Attitude 
            set( h_plot(1), 'XData', X_G, 'YData', Y_G, 'ZData', Z_G)
            xlim( h_ax(1), [-2 2])
            ylim( h_ax(1), [-2 2])
            zlim( h_ax(1), [-2 2])

            set( h_txt(1), 'String', [ 'Time = ', num2str( time, '%0.2f'), ' [s]']);


            %%[*] Magnetic field (Normalized)
            set( h_plot(2), 'UData', mag_vec(1), 'VData', mag_vec(2), 'WData', mag_vec(3))
            xlim( h_ax(2), [-2 2])
            ylim( h_ax(2), [-2 2])
            zlim( h_ax(2), [-2 2])      
    

            drawnow
        end
    end
    
    
    
    if ~ishandle(ButtonHandle)
        disp('Loop stopped by user');
        break;
    end    
    
end


close all hidden

%% close serial port 
fclose( s);
delete( s)
clear s

