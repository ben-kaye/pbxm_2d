import cocos
from cocos import draw
import math
from math import sqrt, atan2, cos, sin
from pbxd import Body, Physics_Engine, Vec2, Lin_Constr, Fixed_Constr
# 2D XPBD Solver

# scene gen:
class render_layer(cocos.layer.Layer):
    def __init__(self):
        super(render_layer, self).__init__()

        # label = cocos.text.Label(
        #     'Hello, world',
        #     font_name = 'Times New Roman',
        #     font_size = 32,
        #     anchor_x = 'center', anchor_y='center'
        # )
        # label.position = 320, 240

        line = draw.Line((0, 0), (0.71, 0.71), (0, 0, 0), 10)

        # self.add(line)
        self.add(line)

# cocos.director.director.init()
# cocos.director.director.run(cocos.scene.Scene(render_layer()))

bodies = list()
# bodies.append(Body(1, Vec2(0.71, 0.71), Vec2(0,0), 1, math.pi/4, 0, 1))
# bodies.append(Body(1, Vec2(2.3, 1.9), Vec2(0,0), 1, math.pi/6, 0, 2))

bodies.append(Body(10, Vec2(5,5), Vec2(0,0), 0, 0, 0, 1))

bodydict = {
    1: 0,
    2: 1,
}

f_ext = {
    1: Vec2(0, -98.1),
}

t_ext = {
    1: 0
}

constraints = list()
# constraints.append(Lin_Constr(bodies[0], bodies[1], Vec2(1, 0), Vec2(-1, 0), 0, 0))
# constraints.append(Fixed_Constr(bodies[0], Vec2(-1, 0), Vec2(0, 0), 0, 0))
constraints.append(Fixed_Constr(bodies[0], Vec2(0,0), Vec2(0, 0), 5, 0))

T = 0
dT = 0.01
SS = 30

while(True):
    Physics_Engine.step(bodies, bodydict, constraints, f_ext, t_ext, 0.01, 30)
    T += dT
    fric = -1*bodies[0].v
    f_ext[1] = fric + Vec2(0, -98.1)
    if bodies[0].p.norm() > 6:
        raise Exception('fucked')
    if bodies[0].v.norm() < 0.1 and bodies[0].p*Vec2(0, -1) > 4.9:
        raise Exception('sim done')


    

input('.')

