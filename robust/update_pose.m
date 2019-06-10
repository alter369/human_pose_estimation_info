function m_alpha = update_pose(x, B, mu, C, L, s, M, t, m_alpha, param)
%update_alpha solves the following optimization problem by ADM
    m_gamma = x - M*(B*m_alpha+mu) - t;
    m_beta = m_alpha;
    lambda1 = zeros(length(x),1);
    lambda2 = zeros(length(m_alpha),1);
    ita = 0.5;
    for i=1:param.max_iter2
        old_gamma = m_gamma;
        m_gamma = update_gamma(x/s, M/s, t/s, B, mu, m_alpha, lambda1, ita);
        
        old_beta = m_beta;
        m_beta  = update_beta (lambda2, m_alpha, ita, param);
        
        old_alpha = m_alpha;
        m_alpha = update_alpha(x/s, B, mu, C, L, M/s, t/s, m_beta, m_gamma, ita, lambda1, lambda2, param);
        
        if(max([norm(old_gamma-m_gamma), norm(old_beta-m_beta), norm(old_alpha-m_alpha), norm(m_beta-m_alpha)]) < param.terminate2)
            break;
        end
        
        lambda1 = lambda1 + ita / 2 * (m_gamma-x+M*(B*m_alpha+mu)+t);
        lambda2 = lambda2 + ita /2 * (m_alpha-m_beta);
    end
end

function m_gamma = update_gamma(x, M, t, B, mu, m_alpha, lambda, ita)
    gamma_dim = length(x);
    cvx_begin quiet
        variable m_gamma(gamma_dim, 1);
        minimize(norm(m_gamma,1)+ita/2*sum((m_gamma-(x-M*(B*m_alpha+mu)-t-lambda/ita)).^2));
    cvx_end
end

function m_beta = update_beta(lambda, m_alpha, ita, param)
    cvx_begin quiet
        variable m_beta(length(m_alpha),1)
        minimize(norm(m_beta,1)+ita/(2*param.theta)*sum((m_beta-(lambda/ita+m_alpha)).^2));
    cvx_end
end

function m_alpha = update_alpha(x, B, mu, C, L, M, t, m_beta, m_gamma, ita, lambda1, lambda2, param)
    nbasis = size(B, 2);    
    W11 = (M*B)'*(M*B) + eye(nbasis);
    W12 = zeros(nbasis, 1);
    W21 = 2*( (m_gamma-x+M*mu+t+lambda1/ita)'*M*B-m_beta'+lambda2'/ita );
    W22 = 0;
    W = [W11 W12; W21 W22];
    
    nconstr = length(L);
    Omega = cell(1, nconstr);
    for i=1:nconstr
        C11 = (C{i}*B)'*(C{i}*B);
        C12 = B'*C{i}'*C{i}*mu;
        C21 = mu'*C{i}'*C{i}*B;
        C22 = mu'*C{i}'*C{i}*mu - L(i);
        Omega{i} = [C11 C12; C21 C22];
    end
    z = [m_beta; 1]; initQ = z * z';
    Q = trace_minimize( W, initQ, Omega, param );
    [V, D] = eig((Q+Q')/2);
    [max_val, max_idx] = max(diag(D));
    z = sqrt(max_val) * V(:, max_idx);
    m_alpha = z(1:end-1) / z(end);
end

