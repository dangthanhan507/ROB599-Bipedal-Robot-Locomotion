function p = gc_kinematics(p)
model = p.robot;
NB = model.NB;

import casadi.*
q = MX.sym('q', NB);
dq = MX.sym('dq', NB);

%% forward kinematics
for ii = 1:NB
    [XJ, S] = jcalc(model.jtype{ii}, q(ii));
    Xup{ii} = XJ * model.Xtree{ii};
    vJ = S * dq(ii);
    if model.parent(ii) == 0
        Xb{ii} = Xup{ii};
        vb{ii} = vJ;
    else
        Xb{ii} = Xup{ii} * Xb{model.parent(ii)};
        vb{ii} = Xup{ii} * vb{model.parent(ii)} + vJ;
    end
end

%% contact points position / velocity / Jacobian
[~,N] = size(model.gc.body);
pos = casadi.MX(3,N);
vel = casadi.MX(3,N);
Jac = casadi.MX(3*N,NB);
dJdq = casadi.MX(3,N);
for jj = 1:N
    ii = model.gc.body(jj);
    X = inv(Xb{ii});
    v = X * vb{ii};
    pt = Xpt(X, model.gc.point(:,jj));
    vpt = Vpt(v, pt);
    J = jacobian(pt,q);
    dJdq_ = reshape(jacobian(J(:),q) * dq, size(J)) * dq;

    pos(:,jj) = pt;
    vel(:,jj) = vpt;
    Jac(3*(jj-1)+(1:3),:) = J;
    dJdq(:,jj) = dJdq_;
end

f_pvj = casadi.Function('pvj', {q, dq}, {pos, vel, Jac, dJdq});
p.f_pvj = f_pvj;

%% com position / velocity
if isfield(model, 'com')
    com = casadi.MX(3,NB);
    vcom = casadi.MX(3,NB);
    for ii = 1:NB
        X = inv(Xb{ii});
        v = X * vb{ii};
        com(:,ii) = Xpt(X, model.com{ii});
        vcom(:,ii) = Vpt(v, com(:,ii));
    end
    pCOM = com * model.m_link(:) / model.mass;
    vCOM = vcom * model.m_link(:) / model.mass;

    f_COM = casadi.Function('COM', {q, dq}, {pCOM, vCOM});
    p.f_COM = f_COM;
end

end