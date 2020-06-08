% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(damping'*square(dq) + slack'*square(delta))
%   subject to
%     J_1(1)*dq(1) + J_1(2)*dq(2) + J_1(3)*dq(3) + J_1(4)*dq(4) + J_1(5)*dq(5) + J_1(6)*dq(6) + J_1(7)*dq(7) == dx(1) + delta(1)
%     J_2(1)*dq(1) + J_2(2)*dq(2) + J_2(3)*dq(3) + J_2(4)*dq(4) + J_2(5)*dq(5) + J_2(6)*dq(6) + J_2(7)*dq(7) == dx(2) + delta(2)
%     J_3(1)*dq(1) + J_3(2)*dq(2) + J_3(3)*dq(3) + J_3(4)*dq(4) + J_3(5)*dq(5) + J_3(6)*dq(6) + J_3(7)*dq(7) == dx(3) + delta(3)
%     J_4(1)*dq(1) + J_4(2)*dq(2) + J_4(3)*dq(3) + J_4(4)*dq(4) + J_4(5)*dq(5) + J_4(6)*dq(6) + J_4(7)*dq(7) == dx(4) + delta(4)
%     J_5(1)*dq(1) + J_5(2)*dq(2) + J_5(3)*dq(3) + J_5(4)*dq(4) + J_5(5)*dq(5) + J_5(6)*dq(6) + J_5(7)*dq(7) == dx(5) + delta(5)
%     J_6(1)*dq(1) + J_6(2)*dq(2) + J_6(3)*dq(3) + J_6(4)*dq(4) + J_6(5)*dq(5) + J_6(6)*dq(6) + J_6(7)*dq(7) == dx(6) + delta(6)
%     J_7(1)*dq(1) + J_7(2)*dq(2) + J_7(3)*dq(3) + J_7(4)*dq(4) + J_7(5)*dq(5) + J_7(6)*dq(6) + J_7(7)*dq(7) == dx(7) + delta(7)
%     qlow(1) <= q0(1) + dt*dq(1)
%     qlow(2) <= q0(2) + dt*dq(2)
%     qlow(3) <= q0(3) + dt*dq(3)
%     qlow(4) <= q0(4) + dt*dq(4)
%     qlow(5) <= q0(5) + dt*dq(5)
%     qlow(6) <= q0(6) + dt*dq(6)
%     qlow(7) <= q0(7) + dt*dq(7)
%     q0(1) + dt*dq(1) <= qup(1)
%     q0(2) + dt*dq(2) <= qup(2)
%     q0(3) + dt*dq(3) <= qup(3)
%     q0(4) + dt*dq(4) <= qup(4)
%     q0(5) + dt*dq(5) <= qup(5)
%     q0(6) + dt*dq(6) <= qup(6)
%     q0(7) + dt*dq(7) <= qup(7)
%     dqlow(1) <= dq(1)
%     dqlow(2) <= dq(2)
%     dqlow(3) <= dq(3)
%     dqlow(4) <= dq(4)
%     dqlow(5) <= dq(5)
%     dqlow(6) <= dq(6)
%     dqlow(7) <= dq(7)
%     dq(1) <= dqup(1)
%     dq(2) <= dqup(2)
%     dq(3) <= dqup(3)
%     dq(4) <= dqup(4)
%     dq(5) <= dqup(5)
%     dq(6) <= dqup(6)
%     dq(7) <= dqup(7)
%     slacklow(1) <= delta(1)
%     slacklow(2) <= delta(2)
%     slacklow(3) <= delta(3)
%     slacklow(4) <= delta(4)
%     slacklow(5) <= delta(5)
%     slacklow(6) <= delta(6)
%     slacklow(7) <= delta(7)
%     delta(1) <= slackup(1)
%     delta(2) <= slackup(2)
%     delta(3) <= slackup(3)
%     delta(4) <= slackup(4)
%     delta(5) <= slackup(5)
%     delta(6) <= slackup(6)
%     delta(7) <= slackup(7)
%
% with variables
%    delta   7 x 1
%       dq   7 x 1
%
% and parameters
%      J_1   7 x 1
%      J_2   7 x 1
%      J_3   7 x 1
%      J_4   7 x 1
%      J_5   7 x 1
%      J_6   7 x 1
%      J_7   7 x 1
%  damping   7 x 1    positive
%    dqlow   7 x 1
%     dqup   7 x 1
%       dt   1 x 1
%       dx   7 x 1
%       q0   7 x 1
%     qlow   7 x 1
%      qup   7 x 1
%    slack   7 x 1    positive
% slacklow   7 x 1
%  slackup   7 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.J_1, ..., params.slackup, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2020-04-23 08:57:30 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
