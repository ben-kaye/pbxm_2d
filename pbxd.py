from math import cos, sin, sqrt, atan2
# from math import pi

class Body:
    def __init__(self, m, p, v, I, q, rate, ID):
        self.p = p
        self.v = v
        self.p_prev = Vec2(0, 0)
        self.m = m
        self.I = I
        self.q = q
        self.rate = rate
        self.ID = ID

class Particle(Body):
    I = 0
    phi = 0
    rate = 0

class Vec2:
    """
    vector of 2 floats
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def normsqd(self):
        return self.x**2 + self.y**2
    def norm(self):
        return sqrt(self.x**2 + self.y**2)
    def rot(self, dq):
        c = cos(dq)
        s = sin(dq)
        return Vec2(c*self.x - s*self.y, s*self.x + c*self.y)
    def arg(self):
        return atan2(self.y, self.x)
    def dir(self):
        # q = self.arg()
        # return Vec2(cos(q), sin(q))
        return self/self.norm()

    def __add__(self, y):
        if isinstance(y, Vec2):
            return Vec2(self.x + y.x, self.y + y.y)
        else:
            return Vec2(self.x + y, self.y + y)

    def __sub__(self, y):
        if isinstance(y, Vec2):
            return Vec2(self.x - y.x, self.y - y.y)
        else:
            return Vec2(self.x - y, self.y - y)

    def __rsub__(self, y):
        if isinstance(y, Vec2):
            return Vec2(y.x - self.x, y.y - self.y)
        else:
            return Vec2(y - self.x, y - self.y)

    def __mul__(self, y):
        if isinstance(y, Vec2):
            return self.x*y.x + self.y*y.y
        else:
            return Vec2(self.x*y, self.y*y)

    def __rmul__(self, y):
        return self*y
    
    def __truediv__(self, y):
        if not isinstance(y, Vec2):
            return Vec2(self.x/y, self.y/y)
        else:
            raise Exception('dividing by vector')

    def __abs__(self):
        return self.norm()

    __radd__ = __add__

        
    @staticmethod
    def uvec(q):
        return Vec2(cos(q), sin(q))
    @staticmethod
    def cross(a, b):
        return a.x*b.y - a.y*b.x
class Constr: # abstract constraint class
    def apply_constr(self, h):
        raise NotImplementedError

class Lin_Constr(Constr):
    """
    constrain two bodies in relative frame for xbpd sim
    """
    def __init__(self, body1, body2, r1, r2, dist, compliance):
        self.dist = dist
        self.dist2 = dist**2
        self.body1 = body1
        self.body2 = body2
        self.r1 = r1
        self.r2 = r2
        self.compliance = compliance
        self.lag = 0
        
    def apply_constr(self, h):
        delta = self.body2.p + self.r2.rot(self.body2.q) - (self.body1.p + self.r1.rot(self.body1.q)) # get deviation from constraint

        comp = self.compliance/h**2 # step normalised compliance

        dlagrang =  -(delta.norm() - self.dist + comp*self.lag )/(1/self.body1.m + 1/self.body2.m + comp)
        self.lag = self.lag + dlagrang

        impulse = dlagrang*delta.dir()

        self.body1.p = self.body1.p + impulse/self.body1.m
        self.body2.p = self.body2.p - impulse/self.body2.m

        # IF BREAKS USE 1/2

        if (self.body1.I != 0 and self.body2.I != 0):
            self.body1.q = self.body1.q + 1/self.body1.I*Vec2.cross(self.r1, impulse)
            self.body2.q = self.body2.q - 1/self.body2.I*Vec2.cross(self.r2, impulse)

class Fixed_Constr(Constr):
    """
    constrain a body to a intertial frame point for xbpd sim
    """

    def __init__(self, body, r, p, dist, comp):
        self.body = body # body id
        self.r = r # body frame offset [vec2]
        self.p = p # world frame point [vec2]
        self.dist = dist # dist {m}
        self.compliance = comp # compliance {m/N}
        self.lag = 0 # lagrangian multiplier

    def apply_constr(self, h):
        fixed_p = self.p
        local_offset = self.r

        delta = (self.body.p + local_offset.rot(self.body.q)) - fixed_p
        comp_norm = self.compliance/h**2

        dlagrang =  -(delta.norm() - self.dist + comp_norm*self.lag)/(1/self.body.m + comp_norm)
        self.lag = self.lag + dlagrang

        # delta_unit = delta.dir()

        impulse = dlagrang*delta.dir()

        self.body.p = self.body.p + impulse/self.body.m

        if self.body.I != 0:
                # IF BREAKS USE 1/2
            self.body.q = self.body.q - 1/self.body.I*Vec2.cross(self.r, impulse) # add moment of inertia (r x n)

class Physics_Engine:
    @staticmethod
    def step(bodies, objdict, constrs, f_ext, t_ext, dt, ss):
        # identify collisions
        
        h = dt/ss

        for i in range(ss): # for each sub step
            
            # make forces here ie friction and stiffness

            for o in bodies: # for each object
                o.p_prev = o.p

                f_b = f_ext.get(o.ID)
                if not f_b is None:
                    o.v = o.v + h*f_b/o.m
                o.p = o.p + h*o.v

                if o.I != 0: # if rigid body
                    o.q_prev = o.q
                    t_b = t_ext.get(o.ID)
                    if not t_b is None:
                        o.rate = o.rate + h*t_b/o.I
                    o.q = o.q + h*o.rate

            for c in constrs:
                c.apply_constr(h)

            for o in bodies:
                o.v = (o.p - o.p_prev)/h
                if o.I != 0:
                    o.rate = (o.q - o.q_prev)/h

