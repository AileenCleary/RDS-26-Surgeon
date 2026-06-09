
def generalized_S(D4, S4):
    S3 = S4[:3, :]
    D3 = D4[:3, :]

    theta_dip = 0.7 * S4[3, :] * D4[3, :]

    S = S3 * D3
    S[2, :] = S[2, :] + theta_dip

    return S

class RouteConfig():
    def __init__(self, D4, S4, desc, name, tendon_names=None, extra=None):
        self.D4 = D4
        self.S4 = S4
        self.desc = desc
        self.name = name
        self.S = generalized_S(D4, S4)
        self.tendon_names = tendon_names




