% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(u_0 + g + f_obs_0, R) + quad_form(x_16 - xN, L_term) + quad_form(u_1 + g + f_obs_1, R) + quad_form(x_16 - xN, L_term) + quad_form(u_2 + g + f_obs_2, R) + quad_form(x_16 - xN, L_term) + quad_form(u_3 + g + f_obs_3, R) + quad_form(x_16 - xN, L_term) + quad_form(u_4 + g + f_obs_4, R) + quad_form(x_16 - xN, L_term) + quad_form(u_5 + g + f_obs_5, R) + quad_form(x_16 - xN, L_term) + quad_form(u_6 + g + f_obs_6, R) + quad_form(x_16 - xN, L_term) + quad_form(u_7 + g + f_obs_7, R) + quad_form(x_16 - xN, L_term) + quad_form(u_8 + g + f_obs_8, R) + quad_form(x_16 - xN, L_term) + quad_form(u_9 + g + f_obs_9, R) + quad_form(x_16 - xN, L_term) + quad_form(u_10 + g + f_obs_10, R) + quad_form(x_16 - xN, L_term) + quad_form(u_11 + g + f_obs_11, R) + quad_form(x_16 - xN, L_term) + quad_form(u_12 + g + f_obs_12, R) + quad_form(x_16 - xN, L_term) + quad_form(u_13 + g + f_obs_13, R) + quad_form(x_16 - xN, L_term) + quad_form(u_14 + g + f_obs_14, R) + quad_form(x_16 - xN, L_term) + quad_form(u_15 + g + f_obs_15, R) + quad_form(x_16 - xN, L_term) - (x_15(6) - theta_mates')*Q)
%   subject to
%     x_1 == A*x_0 + B*(u_0 + g + f_obs_0)
%     x_2 == A*x_1 + B*(u_1 + g + f_obs_1)
%     x_3 == A*x_2 + B*(u_2 + g + f_obs_2)
%     x_4 == A*x_3 + B*(u_3 + g + f_obs_3)
%     x_5 == A*x_4 + B*(u_4 + g + f_obs_4)
%     x_6 == A*x_5 + B*(u_5 + g + f_obs_5)
%     x_7 == A*x_6 + B*(u_6 + g + f_obs_6)
%     x_8 == A*x_7 + B*(u_7 + g + f_obs_7)
%     x_9 == A*x_8 + B*(u_8 + g + f_obs_8)
%     x_10 == A*x_9 + B*(u_9 + g + f_obs_9)
%     x_11 == A*x_10 + B*(u_10 + g + f_obs_10)
%     x_12 == A*x_11 + B*(u_11 + g + f_obs_11)
%     x_13 == A*x_12 + B*(u_12 + g + f_obs_12)
%     x_14 == A*x_13 + B*(u_13 + g + f_obs_13)
%     x_15 == A*x_14 + B*(u_14 + g + f_obs_14)
%     x_16 == A*x_15 + B*(u_15 + g + f_obs_15)
%     x_min <= x_1
%     x_min <= x_2
%     x_min <= x_3
%     x_min <= x_4
%     x_min <= x_5
%     x_min <= x_6
%     x_min <= x_7
%     x_min <= x_8
%     x_min <= x_9
%     x_min <= x_10
%     x_min <= x_11
%     x_min <= x_12
%     x_min <= x_13
%     x_min <= x_14
%     x_min <= x_15
%     x_min <= x_16
%     x_1 <= x_max
%     x_2 <= x_max
%     x_3 <= x_max
%     x_4 <= x_max
%     x_5 <= x_max
%     x_6 <= x_max
%     x_7 <= x_max
%     x_8 <= x_max
%     x_9 <= x_max
%     x_10 <= x_max
%     x_11 <= x_max
%     x_12 <= x_max
%     x_13 <= x_max
%     x_14 <= x_max
%     x_15 <= x_max
%     x_16 <= x_max
%     u_min <= u_0
%     u_min <= u_1
%     u_min <= u_2
%     u_min <= u_3
%     u_min <= u_4
%     u_min <= u_5
%     u_min <= u_6
%     u_min <= u_7
%     u_min <= u_8
%     u_min <= u_9
%     u_min <= u_10
%     u_min <= u_11
%     u_min <= u_12
%     u_min <= u_13
%     u_min <= u_14
%     u_min <= u_15
%     u_0 <= u_max
%     u_1 <= u_max
%     u_2 <= u_max
%     u_3 <= u_max
%     u_4 <= u_max
%     u_5 <= u_max
%     u_6 <= u_max
%     u_7 <= u_max
%     u_8 <= u_max
%     u_9 <= u_max
%     u_10 <= u_max
%     u_11 <= u_max
%     u_12 <= u_max
%     u_13 <= u_max
%     u_14 <= u_max
%     u_15 <= u_max
%
% with variables
%      u_0   4 x 1
%      u_1   4 x 1
%      u_2   4 x 1
%      u_3   4 x 1
%      u_4   4 x 1
%      u_5   4 x 1
%      u_6   4 x 1
%      u_7   4 x 1
%      u_8   4 x 1
%      u_9   4 x 1
%     u_10   4 x 1
%     u_11   4 x 1
%     u_12   4 x 1
%     u_13   4 x 1
%     u_14   4 x 1
%     u_15   4 x 1
%      x_1  16 x 1
%      x_2  16 x 1
%      x_3  16 x 1
%      x_4  16 x 1
%      x_5  16 x 1
%      x_6  16 x 1
%      x_7  16 x 1
%      x_8  16 x 1
%      x_9  16 x 1
%     x_10  16 x 1
%     x_11  16 x 1
%     x_12  16 x 1
%     x_13  16 x 1
%     x_14  16 x 1
%     x_15  16 x 1
%     x_16  16 x 1
%
% and parameters
%        A  16 x 16
%        B  16 x 4
%   L_term  16 x 16   PSD, diagonal
%        Q   2 x 1
%        R   4 x 4    PSD, diagonal
%  f_obs_0   4 x 1
%  f_obs_1   4 x 1
%  f_obs_2   4 x 1
%  f_obs_3   4 x 1
%  f_obs_4   4 x 1
%  f_obs_5   4 x 1
%  f_obs_6   4 x 1
%  f_obs_7   4 x 1
%  f_obs_8   4 x 1
%  f_obs_9   4 x 1
% f_obs_10   4 x 1
% f_obs_11   4 x 1
% f_obs_12   4 x 1
% f_obs_13   4 x 1
% f_obs_14   4 x 1
% f_obs_15   4 x 1
%        g   4 x 1
% theta_mates   2 x 1
%    u_max   4 x 1
%    u_min   4 x 1
%       xN  16 x 1
%      x_0  16 x 1
%    x_max  16 x 1
%    x_min  16 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.x_min, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2018-03-14 07:18:32 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
