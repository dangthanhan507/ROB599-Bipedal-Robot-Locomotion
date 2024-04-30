function [pos, vel, Jac, dJdq] = get_pvj(q, dq, p, idx)

[p_, v_, J_, dJdq_] = p.f_pvj(q, dq);
pos = full(p_(:,idx));
vel = full(v_(:,idx));
Jac = full(J_(3*(idx-1)+(1:3),:));
dJdq = full(dJdq_(:,idx));

end