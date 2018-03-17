from numpy import dot, sum, tile, linalg
from numpy.linalg import inv

### KALMAN FILTER ALGORITHM
### PREDICTION STEP
# INPUT: X(mean state estimate of previous step); P(state covariance of previous step); A(transition n x n matrix); Q(process noise covariance matrix); B(input effect matrix); U(control input)
def predicitionKF(X, P, A, Q, B, U): 
    X = dot(A,X) + dot(B,U) # predict mean X
    P = dot(A, dot(P, A.T)) + Q # predict covariance P
    return (X,P) 

### CORRECTION STEP
# INPUT: X(predicted x matrix); P(predicted P matrix); Y(measurment vector); H(measurement matrix); R(measurement covariance) 
def correctionKF(X, P, Y, H, R):
    V = dot(H,X) # mean of predictive distribution of Y
    S = R + dot(H, dot(P, H.T)) # covariance of Y
    K = dot(P, dot(H.T, inv(S))) # Kalman gain matrix
    X = X + dot(K, (Y - V)) # Update X
    P = P - dot(K, dot(S, K.T)) # Update P
    G = gaussian(Y, V, S) # predictive probability using gaussian function
    return(X, P, K, V, S, G)

# GAUSSIAN FUNCTION
def gaussian(Z, N, U):
    if N.shape()[1] == 1: 
        DX = Z - tile(N, Z.shape()[1])
        E = 0.5 * sum(DX * (dot(inv(U), DX)), axis = 0)
        E = E + 0.5 * N.shape()[0] * log(2*pi) + 0.5 * log(det(U))
        P = exp(-E)
    elif Z.shape()[1] == 1: 
        DX = tile(Z, N.shape()[1]) - N
        E = 0.5 * sum(DX * (dot(inv(U), DX)), axis = 0)
        E = E + 0.5 * N.shape()[0] * log(2 * pi) + 0.5 * log(det(U))
        P = exp(-E)
    else:
        E = 0.5 * dot(DX.T, dot(inv(U), (Z - N)))
        E = E + 0.5 * N.shape()[0] * log(2 * pi) + 0.5 * log(det(U))
        P = exp(-E)
        
    return (P[0], E[0])

        
