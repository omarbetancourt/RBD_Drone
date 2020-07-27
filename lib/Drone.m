classdef Drone < handle
    %% MEMBERS
    properties % of Dynamics
        g
        t
        dt
        tf
        
        m
        l
        I
        
        x              % state vector [X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r]
        r              % position vector [X, Y, Z]
        dr             % velocity vector [dX, dY, dZ]
        euler          % euler angles [phi, theta, psi]
        w              % angular velocity of body [p, q, r]
        
        dx
        
        u              %[T_sum, M1, M2, M3]
        T              % T_sum
        M              % [M1, M2, M3]
    end
        
    properties 
        des_state
        des_r
        des_dr
        des_d2r
        des_yaw 
        
        xacc_c
        yacc_c
        
        phi_des % desired phi angle
        phi_err        % P controller
        phi_err_prev   % D controller
        phi_err_sum    % I controller
        
        theta_des % desired theta angle
        theta_err        % P controller
        theta_err_prev   % D controller
        theta_err_sum    % I controller
        
        psi_des
        psi_err        % P controller
        psi_err_prev   % D controller
        psi_err_sum    % I controller
        
        xpos_des
        xpos_err
        xpos_err_prev
        xpos_err_sum
        
        ypos_des
        ypos_err
        ypos_err_prev
        ypos_err_sum
        
        zpos_des
        zpos_err
        zpos_err_prev
        zpos_err_sum
        
        % gains for the controller. Are modified by drone1_gain dict in
        % main.m
        kP_phi
        kI_phi
        kD_phi
                
        kP_theta
        kI_theta
        kD_theta
                
        kP_psi
        kI_psi
        kD_psi
        
        kP_x
        kI_x
        kD_x
        
        kP_y
        kI_y
        kD_y
        
        kP_z
        kI_z
        kD_z

    end
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = Drone(params, initStates, initInputs, gains, simTime)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.001;
            obj.tf = simTime;
            
            obj.m = params('mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'), 0, 0; %% I matrix
                    0, params('Iyy'), 0; 
                    0, 0, params('Izz')];
            
            %des_state(1:3) = [X; Y; Z] desired positions
            %des_state(4:6) = [dX; dY; dZ] desired velocites
            %des_state(7:9) = [d2X; d2Y; d2Z] desired accelerations
            obj.des_state = zeros(10,1);
            obj.des_r   = obj.des_state(1:3);
            obj.des_dr  = obj.des_state(4:6);
            obj.des_d2r = obj.des_state(7:9);
            obj.des_yaw = obj.des_state(10);
            
            obj.x = initStates;
            % Breaking state vector to smaller vectors for ease of use
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
            
            obj.dx = zeros(12,1);  % vector of derivatives (rates of change) of state variable
            obj.u = initInputs;
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
      
            obj.phi_des = 0.0;
            obj.phi_err = 0.0;     % P controller
            obj.phi_err_prev = 0.0;   % D controller
            obj.phi_err_sum = 0.0;    % I controller

            obj.theta_des = 0.0;
            obj.theta_err = 0.0;        % P controller
            obj.theta_err_prev = 0.0;   % D controller
            obj.theta_err_sum = 0.0;    % I controller

            obj.psi_des = 0.0;
            obj.psi_err = 0.0;        % P controller
            obj.psi_err_prev = 0.0;   % D controller
            obj.psi_err_sum = 0.0;    % I controller
            
            obj.xpos_des = 0.0;
            obj.xpos_err = 0.0;
            obj.xpos_err_sum = 0.0;
            
            obj.xacc_c
            
            obj.ypos_des = 0.0;
            obj.ypos_err = 0.0;
            obj.ypos_err_sum = 0.0; 
            
            obj.yacc_c

            obj.zpos_des = 0.0;
            obj.zpos_err = 0.0;
            obj.zpos_err_sum = 0.0;          

            obj.kP_phi = gains('P_phi');
            obj.kI_phi = gains('I_phi');
            obj.kD_phi = gains('D_phi');

            obj.kP_theta = gains('P_theta');
            obj.kI_theta = gains('I_theta');
            obj.kD_theta = gains('D_theta');

            obj.kP_psi = gains('P_psi');
            obj.kI_psi = gains('I_psi');
            obj.kD_psi = gains('D_psi');
            
            obj.kP_x = gains('P_x');
            obj.kI_x = gains('I_x');
            obj.kD_x = gains('D_x');
            
            obj.kP_y = gains('P_y');
            obj.kI_y = gains('I_y');
            obj.kD_y = gains('D_y');
            
            obj.kP_z = gains('P_z');
            obj.kI_z = gains('I_z');
            obj.kD_z = gains('D_z');
        end
        
        function state = GetState(obj) %% Getter function for state variable
            state = obj.x;
        end
               
        function obj = EvalEOM(obj)
            % Translational Motions
            bRi = RPY2Rot(obj.euler);  %%Roll Pitch Yaw to Rotation
            R = bRi'; % Body-fixed frame to inertial reference frame
            
            obj.dx(1:3) = obj.dr; 
            %Newton's Equation of motion - Acceleration (Eq 3)
            obj.dx(4:6) = 1 / obj.m * ([0; 0; -obj.m*obj.g] + R * obj.T * [0; 0; 1]);
            
            phi = obj.euler(1); theta = obj.euler(2);
            % Rotational Motions - Angular Velocity
            obj.dx(7:9) =[ 1  sin(phi)*tan(theta)  cos(phi)*tan(theta);
                           0  cos(phi)             -sin(phi);
                           0  sin(phi)*sec(theta)  cos(phi)*sec(theta)] * obj.w; % This gives phi_dot, theta_dot, psi_dot (change of euler angles)
            % Angle Accelerations
            obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I *obj.w));
        end
        
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            obj.EvalEOM(); % This function gets dx
            obj.x = obj.x + obj.dx*obj.dt; % Euler method ( can also use Runge-Kutta)
            
            % Updtate other vectors
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
        end
        
        function obj = PositionCtrl(obj, refSig) % Needs position feedback for input
            % output phi, psi, and theta commands (phi_des, psi_des, and
            % theta_des)
            
            % Also output u1 (total thrust)
            
            % Desired position (imported info from refSig in main) will
            % ultimatly come from Tarek's algorithm (nhat)
            obj.xpos_des = refSig(1);
            obj.ypos_des = refSig(2);
            obj.zpos_des = refSig(3);
            
            % Could be attributes: e.g. obj.xvel_des
            xvel_des = obj.des_dr(1);
            yvel_des = obj.des_dr(2);
            zvel_des = obj.des_dr(3);
            
            xacc_des = 0;
            yacc_des = 0;
            zacc_des = 0;
            
            obj.xacc_c = xacc_des + obj.kD_x*(xvel_des - obj.dr(1)) + obj.kP_x*(obj.xpos_des - obj.r(1))
            obj.yacc_c = yacc_des + obj.kD_y*(yvel_des - obj.dr(2)) + obj.kP_y*(obj.ypos_des - obj.r(2))
            zacc_c = zacc_des + obj.kD_z*(zvel_des - obj.dr(3)) + obj.kP_z*(obj.zpos_des - obj.r(3))
            
            obj.u(1) = obj.m*obj.g + obj.m*(zacc_c);
            
            obj.T = obj.u(1);   
        end
        
        function obj  = AttitudeCtrl(obj)
            obj.phi_des   = 1/obj.g*(obj.xacc_c*sin(obj.des_yaw) - obj.yacc_c*cos(obj.des_yaw));
            obj.theta_des = 1/obj.g*(obj.xacc_c*cos(obj.des_yaw) + obj.yacc_c*sin(obj.des_yaw));
            
            p_des = 0;
            q_des = 0;
            r_des = 0;
            
            % Equation 10 Penn
            obj.u(2) = obj.kP_phi * (p_des - obj.w(1)) + obj.kD_phi * (obj.phi_des - obj.euler(1));
            obj.u(3) = obj.kP_theta * (q_des - obj.w(2)) + obj.kD_theta * (obj.theta_des - obj.euler(2));
            obj.u(4) = obj.kP_psi * (r_des - obj.w(3)) + obj.kD_psi * (obj.des_yaw - obj.euler(3));
                   
            
            obj.M = obj.u(2:4);
        end
    end
    
end