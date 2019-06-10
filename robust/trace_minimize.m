function Q = trace_minimize( W, initQ, Omega, param )
%TRACE_MINIMIZE solves the following objective:
%          min. trace(W*Q)
%             s.t. trace(Omega{i}*Q) = 0, i=1,...,m
%             s.t. Q >= 0, rank(Q) <= 1
    Q = initQ; P = Q;
    G = zeros(size(Q)) + rand(size(Q))*0.00000001;
    for i=1:param.max_iter3
        old_Q = Q;
        Q = update_Q(W, P, G, Omega, param.delta);
        
        old_P = P;
        P = update_P(Q, G, param.delta);
        
        if max([norm(old_Q-Q), norm(old_P-P), norm(P-Q)]) < param.terminate3
            break;
        end
        %disp(([norm(old_Q-Q), norm(old_P-P), norm(P-Q)]));
        G = G + 0.5 * (Q-P);
    end
end

function Q = update_Q(W, P, G, Omega, delta)

    cvx_begin quiet
        variable Q(size(P)) symmetric;
        minimize(trace(W*Q)+trace(G'*Q)+delta/2*sum(sum((Q-P).^2)));
            subject to
                for i=1:length(Omega)
                    trace(Omega{i}*Q) == 0;
                end
                Q(end,end)==1;
    cvx_end
end

function P = update_P(Q, G, delta)
    Q_hat = Q + 2 / delta * G;
    Q_hat = (Q_hat + Q_hat') / 2;
    [V, D] = eig(Q_hat);
    [max_val, max_idx] = max(diag(D));
    if max_val >= 0
        P = max_val * V(:, max_idx) * V(:, max_idx)';
    else
        P = zeros(size(Q));
    end
end











