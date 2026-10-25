
rotation about x
Rmodel = [1 0 0;
          0 0 -1;
          0 1  0];

% X remains unchanged.
% 
% Y rotates toward Z.
% 
% Z rotates toward −Y.
% 
% Use this if the model is rolled sideways relative to the expected orientation.




rotation about y

Rmodel = [ 0 0 1;
    0 1 0;
    -1 0 0];

% Y remains unchanged.
% 
% X rotates toward −Z.
% 
% Z rotates toward X.
% 
% Use this if the model's nose or longitudinal axis points in the wrong direction verticall


rotation about z
Rmodel = [0 -1 0;
    1  0 0;
    0  0 1];

% Z remains unchanged.
% 
% X rotates toward Y.
% 
% Y rotates toward −X.
% 
% Use this if the aircraft points sideways instead of forward.