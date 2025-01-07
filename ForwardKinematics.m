% DH parameters for ABB IRB 120
a = [0 270 70 0 0 0];   % link lengths
alpha = [-pi/2 0 -pi/2 pi/2 -pi/2 0];   % link twists
d = [290 0 0 302 0 72];   % link offsets
theta = [0 -pi/2 0 0 0 0];   % joint angles

% Create a 4x4 transformation matrix for each joint
T = cell(1, 6);  % Create an empty cell array to store transformation matrices
for i = 1:6
    % Construct the 4x4 transformation matrix from DH parameters for the i-th joint
    T{i} = [cos(theta(i)), -sin(theta(i)) * cos(alpha(i)), sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
            sin(theta(i)), cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
            0, sin(alpha(i)), cos(alpha(i)), d(i);
            0, 0, 0, 1];
end

% Calculate the forward kinematics by multiplying all the transformation matrices
T_0_6 = T{1} * T{2} * T{3} * T{4} * T{5} * T{6};

% Extract the position and orientation from the forward kinematics matrix
position = T_0_6(1:3, 4);  % The first three elements of the last column are the position
orientation = rotm2eul(T_0_6(1:3, 1:3), 'XYZ');  % Convert the 3x3 rotation matrix to Euler angles

% Display the results
disp("Forward Kinematics (without Robotics Toolbox):");
disp("Position (mm):");
disp(position');
disp("Orientation (rad):");
disp(rad2deg(orientation)');
disp(T_0_6);
