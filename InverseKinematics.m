% Define the robot links
L(1) = Link([0,     290,    0,       -pi/2]);
L(2) = Link([-pi/2, 0 ,     270,     0    ]);
L(3) = Link([0,     0 ,     70,      -pi/2]);
L(4) = Link([0,     302,    0 ,      pi/2]);
L(5) = Link([0,     0 ,     0 ,      -pi/2]);
L(6) = Link([0,     72,     0 ,      0  ]);

% Create the robot model
IRB_120 = SerialLink(L);

% Define the target points
point(1, :) = [374  0    630];
point(2, :) = [0  450    580];
point(3, :) = [0  450  285];
point(4, :) = [0  450    580];
point(5, :) = [400  0  580];
point(6, :) = [400  0  461];
point(7, :) = [400  0  580];
point(8, :) = [100  0  580];
point(9, :) = [400  0  580];
point(10, :) = [400  0  461];
point(11, :) = [400  0  580];
point(12, :) = [0 -450  580];
point(13, :) = [0  -450  281];
point(14, :) = [374  0    630];

samples = 1;

% Initialize variables for x, y, and z coordinates
x = zeros((samples*(length(point)-1))+1, 1);
y = zeros((samples*(length(point)-1))+1, 1);
z = zeros((samples*(length(point)-1))+1, 1);

% Generate trajectory points
for trans = 1:length(point)-1
    if point(trans+1,1)-point(trans,1) == 0
        x(((trans-1)*samples)+1:trans*samples+2,1) = point(trans,1);
    else
        xstore = point(trans,1):((point(trans+1,1)-point(trans,1))/samples):point(trans+1,1);
        if trans > 1
            x(((trans-1)*samples)+2:trans*samples+2,1) = xstore';
        else
            x(((trans-1)*samples)+1:trans*samples+1,1) = xstore';
        end
    end
end

for trans = 1:length(point)-1
    if point(trans+1,2)-point(trans,2) == 0
        y(((trans-1)*samples)+1:trans*samples+2,1) = point(trans,2);
    else
        ystore = point(trans,2):((point(trans+1,2)-point(trans,2))/samples):point(trans+1,2);
        if trans > 1
            y(((trans-1)*samples)+2:trans*samples+2,1) = ystore';
        else
            y(((trans-1)*samples)+1:trans*samples+1,1) = ystore';
        end
    end
end

for trans = 1:length(point)-1
    if point(trans+1,3)-point(trans,3) == 0
        z(((trans-1)*samples)+1:trans*samples+2,1) = point(trans,3);
    else
        zstore = point(trans,3):((point(trans+1,3)-point(trans,3))/samples):point(trans+1,3);
        if trans > 1
            z(((trans-1)*samples)+2:trans*samples+2,1) = zstore';
        else
            z(((trans-1)*samples)+1:trans*samples+1,1) = zstore';
        end
    end
end

% Initialize the transformation matrices
T = zeros(4, 4, length(x));
for loop = 1:length(x)
    T(:,:,loop) = [1,  0,  0, 0;
                   0, -1,  0, 0;
                   0,  0, -1, 0;
                   0,  0,  0, 1];
end

% Set the position values in the transformation matrices
for loop = 1:length(x)
    T(1,4,loop) = x(loop);
    T(2,4,loop) = y(loop);
    T(3,4,loop) = z(loop);
end

% Perform inverse kinematics for each pose
q1 = zeros(length(x), 6);
for loop = 1:length(x)
    q_sol = IRB_120.ikine(T(:,:,loop), 'mask', [1 1 1 0 0 0]);
    if ~any(isnan(q_sol))
        q1(loop,:) = q_sol;
    else
        fprintf('No valid solution found for pose %d.\n', loop);
    end
end

q1=rad2deg(q1);
disp(q1)