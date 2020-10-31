import pygame

import math
from math import sqrt, atan2, cos, sin
# 2D XPBD Solver

class obj:
    def __init__(self, m, p, v, I, phi, rate, ID):
        self.p = p
        self.v = v
        self.p_prev = vec2(0, 0)
        self.m = m
        self.I = I
        self.phi = phi
        self.rate = rate
        self.ID = ID

    def set_p(self, p):
        self.p_prev = self.p
        self.p = p
    def set_p_prev(self, p_prev):
        self.p_prev = p_prev


class particle(obj):
    I = 0
    phi = 0
    rate = 0

class vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, y):
        return vec2(self.x + y.x, self.y + y.y)
    def __sub__(self, y):
        return vec2(self.x - y.x, self.y - y.y)
    def __mul__(self, y):
        return self.x*y.x + self.y*y.y
    def __rmul__(self, y):
        return vec2(self.x*y, self.y*y)
        
    def normsqd(self):
        return self.x**2 + self.y**2
    def norm(self):
        return sqrt(self.x**2 + self.y**2)
    def rot(self, dq):
        c = cos(dq)
        s = sin(dq)
        return vec2(c*self.x - s*self.y, s*self.x + c*self.y)
    def arg(self):
        return atan2(self.y, self.x)
    def dir(self):
        q = self.arg
        return vec2(cos(q), sin(q))
        
    @staticmethod
    def uvec(q):
        return vec2(cos(q), sin(q))

class constr:
    def apply_constr(self, constrs, objs, objdict, h):
        raise NotImplementedError

class lin_constr(constr):
    def __init__(self, b1id, b2id, r1, r2, dist, comp):
        self.dist = dist
        self.dist2 = dist**2
        self.b1id = b1id
        self.b2id = b2id
        self.r1 = r1
        self.r2 = r2
        self.comp = comp
        self.lag = 0
        
    def apply_constr(self, constrs, objs, objdict, h):
        c = self

        b1 = objs[objdict.get(c.b1id)]
        b2 = objs[objdict.get(c.b2id)]
        
        if (b1.I != 0 and b2.I != 0):

            p1 = b1.p
            p2 = b2.p

            dp = p2 + c.r2.rot(b2.q) - (p1 + c.r1.rot(b1.q))
            comp = c.comp/h**2

            dlag =  (- dp.norm() - comp*c.lag )/(1/b1.m + 1/b2.m + comp)
            c.lag = c.lag + dlag

            imp = dlag*dp.dir()

            b1.p = b1.p + imp/b1.m1
            b2.p = b2.p - imp/b2.m2


            # IF BREAKS USE 1/2
            b1.q = b1.q + 1/b1.I*(b1.x*imp.y - b1.y*imp.x)
            b1.q = b2.q + 1/b2.I*(b2.x*imp.y - b2.y*imp.x)
    


class fixed_constr(constr):
    def __init(self, bid, r, dist, comp):
        self.bid = bid
        self.r = r
        self.p = p
        self.dist = dist
        self.comp = comp
        self.lag = 0

    def apply_constr(self, constrs, objs, objdict, h):
        c = self

        b = objs[objdict.get(c.bid)]
        
        if b.I != 0:
            p1 = b1.p
            p2 = b2.p

            dp = p2 + c.r2.rot(b2.q) - (p1 + c.r1.rot(b1.q))
            comp = c.comp/h**2

            dlag =  (- dp.norm() - comp*c.lag )/(1/b1.m + 1/b2.m + comp)
            c.lag = c.lag + dlag

            imp = dlag*dp.dir()

            b1.p = b1.p + imp/b1.m1
            b2.p = b2.p - imp/b2.m2


            # IF BREAKS USE 1/2
            b1.q = b1.q + 1/b1.I*(b1.x*imp.y - b1.y*imp.x)
            b1.q = b2.q + 1/b2.I*(b2.x*imp.y - b2.y*imp.x)

class physim:
    @staticmethod
    def sim(objs, objdict, constrs, f_ext, t_ext, dt, ss):
        # identify collisions
        
        h = dt/ss

        for i in range(ss): # for each sub step
            
            # make forces here ie friction and stiffness

            for o in objs: # for each object
                o.p_prev = o.p
                o.v = o.v + h*f_ext.get(o.ID)/o.m
                o.p = o.p + h*o.v

                if o.I != 0: # if rigid body
                    o.q_prev = o.q
                    o.rate = o.rate + h*t_ext.get(o.ID)/o.I
                    o.q = o.q + h*o.rate

            for c in constrs:
                c.apply_constr(constrs, objs, objdict, h)

            for o in objs:
                o.v = (o.p - o.p_prev)/h
                o.rate = (o.q - o.q_prev)/h


        


# scene gen:

objs = list()
objs.append(obj(1, vec2(0,0), vec2(0,0), 1, math.pi/4, 0, 1))
objs.append(obj(1, vec2(1,0), vec2(0,0), 1, -math.pi/3, 0, 2))

constraints = list()
constraints.append(lin_constr(1, 2, vec2(0, 1), vec2(0, 1), 0, 0))


