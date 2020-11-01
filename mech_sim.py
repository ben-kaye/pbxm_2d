import cocos
from cocos import draw
import math
from math import sqrt, atan2, cos, sin
from pbxd import body, physics_engine, vec2, lin_constr, fixed_constr
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
bodies.append(body(1, vec2(0.71, 0.71), vec2(0,0), 1, math.pi/4, 0, 1))
bodies.append(body(1, vec2(2.3, 1.9), vec2(0,0), 1, math.pi/6, 0, 2))



bodydict = {
    1: 0,
    2: 1,
}

f_ext = {
    2: vec2(0, -9.81),
}

t_ext = {
    2: 0
}

constraints = list()
constraints.append(lin_constr(1, 2, vec2(1, 0), vec2(-1, 0), 0, 0))
constraints.append(fixed_constr(1, vec2(-1, 0), vec2(0, 0), 0, 0))

while(True):
    physics_engine.sim(bodies, bodydict, constraints, f_ext, t_ext, 0.01, 30)
    if bodies[1].p.norm() > 3.5:
        raise Exception('fucked')

    

input('.')

