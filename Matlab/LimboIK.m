function th = LimboIK(O7z, psi_x, psi_y)
% O7z in cm, psi_x and psi_y in deg
% th has solution pairs given as
% | thA1 thB1 thC1 |
% | thA2 thB2 thC2 |

  th = zeros(2,3);

  % Constants, all in cm
  L = [7.5, 8.5]; % Link lengths
  B = 8.5;      % Base radius
  P = 9.3;      % Plate radius (to link attach. points)

  % Yaw, eq. (3.13)
  psi_z = atan2d(-sind(psi_x) * sind(psi_y), cosd(psi_x) + cosd(psi_y));

  % Rotation matrix, eq. (3.12)
  R = Rx(psi_x)*Ry(psi_y)*Rz(psi_z);
  u = R(:,1); v = R(:,2); w = R(:,3);

  % Vector from base origin to plate origin
  O7x = P*(u(1) - v(2)) / 2; % eq. (3.11)
  O7y = -u(2)*P;             % eq. (3.7)
  r_P = [O7x O7y O7z]';      % eq. (3.2)

  % Calculating the angles
  for m = 0:2 % A, B, C links

    alpha_1m = 120*m;
     
    % Clean up the A_i, B_i, and C_i equations
    ca_1m = cosd(alpha_1m);
    
    % Vector from plate origin to manipulator attach. point
    p_j = R*Rz(alpha_1m)*[P 0 0]';
    
    % Point at which manipulator is attached relative to origin
    O7j = r_P + p_j;
    
    % Some coefficients
    Am = 2*L(1)*ca_1m*(B*ca_1m - O7j(1));
    Bm = 2*L(1)*O7j(3)*(ca_1m^2);
    Cm = O7j(1)^2 - 2*B*O7j(1)*ca_1m + ...
           ca_1m^2 * (B^2 + L(1)^2 - L(2)^2 + O7j(3)^2);
    
    % Desired angles given the configuration
    if Am^2 + Bm^2 - Cm^2 < 0
      th(:,m+1) = [NaN, NaN]';
    else
      th(:,m+1) = 2*atan2d(-Bm + [-1 1].*sqrt(Am^2 + Bm^2 - Cm^2), Cm - Am);
    end

  end
  
end

function rx = Rx(th)
rx = [1       0         0   ;
      0  cosd(th) -sind(th) ;
      0  sind(th)  cosd(th)];
end

function ry = Ry(th)
ry = [ cosd(th)  0  sind(th) ;
            0    1       0   ;
      -sind(th)  0  cosd(th)];
end

function rz = Rz(th)
rz = [ cosd(th) -sind(th) 0 ;
       sind(th)  cosd(th) 0 ;
            0         0   1];
end