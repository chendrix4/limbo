% ASSIGNMENT07
% Lothar Rubusch
% 2013-Apr-29

% cleanup
clear; clc; close all;

% Implement a potential field navigation strategy with round round obstacles and
% the repulsing and attractive functions given in the previous lecture. The
% environment has the size 200 x 300 cm, the obstacles a diameter of 15cm.
%
%
% Define a scenario with 9 obstacles, a starting position and a goal position.
% Let a holonomic robot (with a diameter of 7cm) navigate from the start to the
% goal position by moving down the gradient of the potential field. Choose an
% adequate step size for the gradient descent. Plot the paths of the robot and
% the potential fields for different configurations.
%
%
% Use a point for the robot and extend the diameter of the obstacles by the
% diameter of the robot.
%
%
% It is not possible to reach the goal in each configuration. Setup a situation
% where the robot cannot reach the goal with the potential field method.
%
%
% To validade that the robot is not colliding with the obstacles visualize the
% obstacles in the path plot. You can use 'pdcirc' in a 2D plot and 'patch' in a
% 3D plot to visualize the obstacle in MATLAB.
%
%
%
%% Formulae
%
% potential field
% U_att(q) = 1/2 * k(att) * rho_goal^2(q)
%
% attracting force converges linearily towards 0 (goal)
% F_att(q) = -nabla(U_att( q ))
%          = -k_att * rho_goal( q ) * nabla( rho_goal( q ) )
%          = -k_att * (q - q_goal)
%
% repulsing potential field
%             / 1/2 * k_rep * ( 1 / rho(q) - 1/rho_zero )^2 ;  if rho(q) <  rho_0
% U_rep(q) = {
%             \ 0                                        ;  if rho(q) >= rho_0
%
%            ; p(q) is the minimum distance between q and the object
%            ; k_rep = k_att
%
% F_rep(q) = -nabla( U_rep(q) )
%
%              / k_rep( 1/rho(q) - 1/rho_0 ) * 1/(rho( p ))^2 * (q - q_obstacle) / rho( q )
%             /                                          ; if rho( q ) < rho_0
%          = {
%             \ 0                                        ; if rho( q ) >= rho_0
%

         %% START                                                                        


%% set up field
%XMAX=60
%YMAX=40
XMAX=300
YMAX=200
M=zeros(YMAX, XMAX);
goal = [ YMAX/2 XMAX/2 ];
start = [ 1 1 ];
ty = [1:size(M, 1)];
tx = [1:size(M, 2)];

% %% repulsive potential field
% M = repulsive_potential_field( M );
% M = normalize( M );
% figure;
% mesh( tx, ty, M );
% title('Repulsive Potential Field\n');

%% attractive potential field
M = attractive_potential_field( M, goal );
M = normalize( M );
figure
mesh( tx, ty, M );
title('Attractive Potential Field\n');

%% visualization
figure
mesh( tx, ty, M );
title('Path towards goal\n');
hold on
solution = solve( M, start, goal );
plot3( solution(:,2), solution(:,1), solution(:,3)+0.1, 'r.' );
hold off


%% return the euclidian distance between 2 points
%% src - one distinct point: [y x]
%% dst - coords of obstacle / destination:
%% [ y1 x1;
%%   y2 x2;
%%   y3 x3 ]
%%
%% returns 0 if point is inside the area of dst
function [ dist ] = distance( src, dst )
  dists = [];
  for i=1:size( dst , 1)
    dx=0; dy=0;
    dy = src(1) - dst(i,1);
    dx = src(2) - dst(i,2);
    dists(i) = sqrt( dy^2 + dx^2 );
  end
  dist = min( dists );
  end

%% function to normalize values between 0.0 and 1.0
function [ M_normalized ] = normalize( M_raw )
  M_normalized = 1/max(max( M_raw )) * M_raw;
end

%% backtrack for solver
function [ ret ] = isinbacktrack( backtrack, y, x )
  for( i=1:size(backtrack, 1) )
    if( y == backtrack(i,1) )
      if( x == backtrack(i,2) )
        ret = 1;
        return;
      end
    end
  end
  ret = 0;
end

%% solver for path
function [ solution ] = solve( M, start, goal )
  % init
  y = start(1);
  x = start(2);
  solution = [ y x M(y,x)];
  backtrack = [];
  idxx = 1;
  U = 0;

  while 1
    backtrack = [ backtrack ; y x ];

    y = solution(idxx,1);
    x = solution(idxx,2);
    idxx = idxx + 1;

    U = M(y,x);

    %% scan
    if( (size(M, 1) <= y) || 1 == isinbacktrack( backtrack, y+1, x) )
      U_up    = 10;
    else
      U_up    = M( y+1, x );
    end

    if( (1 >= y) || 1 == isinbacktrack( backtrack, y-1, x) )
      U_down  = 10;
    else
      U_down  = M( y-1, x );
    end

    if( (1 >= x) || 1 == isinbacktrack( backtrack, y, x-1) )
      U_left  = 10;
    else
      U_left  = M( y, x-1 );
    end

    if( (size(M,2) <= x) || 1 == isinbacktrack( backtrack, y, x+1) )
      U_right = 10;
    else
      U_right = M( y, x+1 );
    end

    %% find x_next and y_next (smallest val in M)
    U_next = min( [ U_up U_down U_left U_right ] );
    if( U_next == U_down )
      y = y-1;
    elseif( U_next == U_right )
      x = x+1;
    elseif( U_next == U_up )
      y = y+1;
    elseif( U_next == U_left )
      x = x-1;
    else
      break
    end

    %% if x and y is goal, break
    if( y == goal(1,1) && x == goal(1, 2) )
      break;
    end
    solution = [ solution ; y x M(y,x) ];
  end
end


%% methods


%% place an obstacle
function [ M_ng ] = set_obstacle( M, obs )
  k_rep = 5;
  rho_zero = 10;
  U_rep = 0; U_obs = 0;
  for yval=1:size(M,1)
    for xval=1:size(M,2)
      dobs = distance( [yval xval], obs);
      if ( 0 == dobs )
        U_rep = k_rep;
      else
        U_rep = M(yval,xval) + 1/2 * k_rep * ( 1/dobs - 1/rho_zero )^2;
        if k_rep < U_rep
          U_rep = k_rep;
        end
      end
      M_ng(yval,xval) = U_rep;
    end
  end
end

%% repulsive potential field
function [ M_ng ] = repulsive_potential_field( M )
  fact=5
  obs1 = [ 26 44; 25 45 ; 26 45; 27 45; 26 46 ];
  M = set_obstacle( M, obs1 );

  obs2 = fact * [ 16 4; 15 5 ; 16 5; 17 5; 16 6 ];
  M = set_obstacle( M, obs2 );

  obs3 = fact * [ 36 4; 35 5 ; 36 5; 37 5; 36 6 ];
  M = set_obstacle( M, obs3 );

  obs4 = fact * [ 6 14; 5 15 ; 6 15; 7 15; 6 16 ];
  M = set_obstacle( M, obs4 );

  obs5 = fact * [ 16 24; 15 25 ; 16 25; 17 25; 16 26 ];
  M = set_obstacle( M, obs5 );

  obs6 = fact * [ 16 54; 15 55 ; 16 55; 17 55; 16 56 ];
  M = set_obstacle( M, obs6 );

  obs7 = fact * [ 36 54; 35 55 ; 36 55; 37 55; 36 56 ];
  M = set_obstacle( M, obs7 );

  obs8 = fact * [ 10 43; 11 42 ; 11 43; 12 44; 11 45 ];
  M = set_obstacle( M, obs8 );

  obs9 = fact * [ 26 22; 25 23 ; 26 23; 27 23; 26 24 ];
  M = set_obstacle( M, obs9 );

  M_ng = M;
end

%% attractive potential field
function [ M_ng ] = attractive_potential_field( M, goal )
  k_att = 10;
  U_att = 0;
  for yval=1:size(M,1)
    for xval=1:size(M,2)
      %% base potential
      rho_goal = distance( [yval xval], goal );

      %% attractive
      U_att = 1/2 * k_att * rho_goal^2;

      %% potential field
      U(yval, xval) = U_att;
    end
  end
  U = normalize( U );
  M_ng = M + U;
end
