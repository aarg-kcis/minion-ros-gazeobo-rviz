% Produced by CVXGEN, 2018-03-14 07:19:08 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: make_csolve.m.
% Description: Calls mex to generate the csolve mex file.
%mex -v csolve.c ldl.c matrix_support.c solver.c util.c
mex csolve.c ldl.c matrix_support.c solver.c util.c
