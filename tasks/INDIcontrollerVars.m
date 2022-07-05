%% INDI Controller Variables

% Attitude Control
b = Vehicle.Airframe.d / 2;
l = Vehicle.Airframe.d / 2;
k1 = Vehicle.Rotor.w2ToThrustGain;
k2 = Vehicle.Rotor.Cq/Vehicle.Rotor.Ct*Vehicle.Rotor.radius;
I_rzz = Vehicle.Rotor.inertia;
I_v_inv = inv(Vehicle.Airframe.inertia);

INDI.Mc = [-b*k1 b*k1 b*k1 -b*k1;
      l*k1 l*k1 -l*k1 -l*k1;
      k2 -k2 k2 -k2;];

INDI.G1 = 2\I_v_inv * INDI.Mc;


INDI.G2 = I_v_inv * (1/Ts) * [0 0 0 0; 0 0 0 0; I_rzz -I_rzz I_rzz -I_rzz];

% G3 = I_v_inv * [I_rzz -I_rzz I_rzz -I_rzz; -I_rzz I_rzz -I_rzz I_rzz; 0 0 0 0];

INDI.G1G2_inv = pinv(INDI.G1+INDI.G2);