import pygame

import math
from math import sqrt, atan2, cos, sin

from pbxd import obj, physics_engine, vec2, lin_constr, fixed_constr
# 2D XPBD Solver

# scene gen:

objs = list()
idx1 = objs.append(obj(1, vec2(0,0), vec2(0,0), 1, math.pi/4, 0, 1))
idx2 = objs.append(obj(1, vec2(1,0), vec2(0,0), 1, -math.pi/3, 0, 2))

objdict = {
    "1": idx1,
    "2": idx2,
}

f_ext = {
    "2": vec2(0, -9.81),
}
t_ext = {

}

constraints = list()
constraints.append(lin_constr(1, 2, vec2(0, 1), vec2(0, 1), 0, 0))
constraints.append(fixed_constr(1, vec2(0, -1), vec2(0, 0), 0, 0))

while(True):
    physics_engine.sim(objs, objdict, constraints, f_ext, t_ext, 0.1, 10)


