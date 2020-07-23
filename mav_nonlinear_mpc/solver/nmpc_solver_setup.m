clc;
clear all;
close all;

Ts = 0.1; %prediction sampling time
EXPORT = 1;

DifferentialState velocity(3) roll pitch yaw position(3) rollrate_ext pitchrate_ext;
Control roll_ref pitch_ref thrust;

OnlineData roll_tau;
OnlineData roll_gain;
OnlineData pitch_tau;
OnlineData pitch_gain;
OnlineData linear_drag_coefficient(2);
OnlineData external_forces(3);
OnlineData external_torques(2);

OnlineData target_position(3);
OnlineData target_velocity(3);

OnlineData impact_force;

n_XD = length(diffStates);
n_U = length(controls);

g = [0;0;9.8066];
e = 2.718;


R = [cos(yaw)*cos(pitch)  cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw)  (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)); ...
    cos(pitch)*sin(yaw)  cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll)  (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)); ...
    -sin(pitch)                            cos(pitch)*sin(roll)                            cos(pitch)*cos(roll)];

x_B = R(:,1);
y_B = R(:,2);
z_B = R(:,3);

%% Impact function

pos_diff = position - target_position;
x_impact = (pos_diff)' * x_B;
y_impact = (pos_diff)' * y_B;
z_impact = (pos_diff)' * z_B;

r_impact = x_impact^2 + y_impact^2;

h_impact_location = e^(-(r_impact*111-5.1))/(1+e^(-(r_impact*111-5.1)))+ (x_impact+y_impact);%1/(1+e^(x_impact*25-6));  %1/(1+e^(-x_impact/0.05-3)); 3.2* r_impact^3 + (1+e^(-r_impact/0.009-5.1))

%% Impact Predictor

%Activation Functions with Maximum = 1
act_x = 1 / (1 + e^((x_impact-0.7)*50)) * (1 + e^((-x_impact-0.7)*50));
act_y = 1 / (1 + e^((y_impact-0.7)*50)) * (1 + e^((-y_impact-0.7)*50));

% v_diff = velocity - target_velocity;
% vz_diff = v_diff' * z_B;
% pred_z_force = 1 * vz_diff / (4.6 + 1) * impact_act_t * act_r;
% pred_y_torque = -1 * vz_diff * x_impact / 0.192 * impact_act_t * act_x;
% pred_x_torque = -1 * vz_diff * y_impact / 0.185 * impact_act_t * act_y;
pred_y_torque = -x_impact * impact_force * act_x;
pred_x_torque = y_impact * impact_force * act_y;

%% Differential Equation

%nonlinear drag model
drag_acc = thrust*[linear_drag_coefficient1 0 0; 0 linear_drag_coefficient2 0; 0 0 0]*R'*velocity;

droll = (1/roll_tau)*(roll_gain*roll_ref - roll) + rollrate_ext;
dpitch = (1/pitch_tau)*(pitch_gain*pitch_ref - pitch) + pitchrate_ext;

f = dot([velocity; roll; pitch; yaw; position; rollrate_ext; pitchrate_ext]) == ...
    [z_B*thrust-g-drag_acc+external_forces;...
    droll; ...
    dpitch;...
    0;...
    velocity;...
    -external_torques1 - pred_x_torque;...
    external_torques2 + pred_y_torque;...
    ];

h = [position;...
    velocity;...
    roll;...
    pitch;...
    roll_ref;...
    pitch_ref;...
    z_B(3)*thrust-g(3);...
    h_impact_location];

hN = [position;...
    velocity;...
    h_impact_location];

%% MPCexport
acadoSet('problemname', 'mav_position_mpc');

N = 20;
ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(length(h));
WN_mat = eye(length(hN));
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );
ocp.subjectTo(-deg2rad(45) <= [roll_ref; pitch_ref] <= deg2rad(45));
ocp.subjectTo( g(3)/2.0 <= thrust <= g(3)*1.5);
ocp.setModel(f);


mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',        'FULL_CONDENSING_N2'  ); %FULL_CONDENsinG_N2
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',         N                  );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT',          1e-10				);
mpc.set( 'LINEAR_ALGEBRA_SOLVER',        'GAUSS_LU'         );
mpc.set( 'IMPLICIT_INTEGRATOR_NUM_ITS',  2                  );
mpc.set( 'CG_USE_OPENMP',                'YES'              );
mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES', 'NO'              );
mpc.set( 'CG_USE_VARIABLE_WEIGHTING_MATRIX', 'NO'           );


if EXPORT
    mpc.exportCode('.');
end
