gps_file = dlmread('gnss.csv');
odom_file = dlmread('odom.csv');
ground_truth = dlmread('ground_truth.csv');

figure; 
hold on;
plot(ground_truth(:, 2),ground_truth(:, 3))
hold off;

t_gt = ground_truth(:, 2);
position_x = ground_truth(:, 2);
position_y = ground_truth(:, 3);
orientation_z = ground_truth(:, 6);
orientation_w = ground_truth(:, 7);
speed_x = ground_truth(:, 8);
angular_z = ground_truth(:, 9);



linear_x = odom_file(:, 2);
odom_angular_z = odom_file(:, 4);
cov_linear_x = odom_file(:, 5);
cov_linear_y = odom_file(:, 6);
cov_angular_z = odom_file(:, 7);


figure; 
hold on;
plot(gps_file(:, 2),gps_file(:, 3))
hold off;

gnss_position_x = gps_file(:, 2);
gnss_position_y = gps_file(:, 3);


#system_state_x
x = [position_x(2) position_y(2) angular_z(2)]';

#Covariance_matrix_P
P = [ [cov_linear_x(2) 0 0],
                        [0 cov_linear_y(2) 0],
                        [0 0 cov_angular_z(2)]];
                        
#Dynamics_matrix_A
A = [ [speed_x(2) 0 (t_gt(3) - t_gt(2)) 0],
                      [0 1 0 (t_gt(3) - t_gt(2))],
                      [0 0 1 0],
                      [0 0 0 1]];

       
#Process noise co-variance matrix Q???
       
#ControlInputU
u = [linear_x(2) odom_angular_z(2)]';

#Measuring matrix H
H = 

#Measurement noise covariance matrix R
R = [ [cov_linear_x(2) 0 0],
                        [0 cov_linear_y(2) 0],
                        [0 0 cov_angular_z(2)]];


for n = 3: size(position_x)-2
     x = A*x
     P = A*P*A' + Q
 
 # Measurement Update (Correction)
 # ===============================
 # Compute the Kalman Gain
    S = H*P*H' + R
    K = (P*H') * np.linalg.pinv(S)# Update the estimate via z
    Z = mx(n)
    y = Z - (H*x) # Innovation or Residual
    x = x + (K*y)
 
    I = [[1 0 0], [0 1 0], [0 0 1]];
 # Update the error covariance
    P = (I - (K*H))*P
    
    
end    