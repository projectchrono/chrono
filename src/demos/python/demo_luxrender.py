#-------------------------------------------------------------------------------
# Name:        modulo1
# Purpose:
#
# Author:      tasora
#
# Created:     13/02/2012
# Copyright:   (c) tasora 2012
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()

import pylux
import os

context = pylux.Context('my renderer')
context.film('multiimage',[('filename','testout5.png'),('xresolution',320),('yresolution',240)]);
context.worldBegin()


context.sampler('metropolis', [])
context.surfaceIntegrator('path', [('string lightstrategy', 'one')])
context.renderer('hybrid', [])

context.lookAt(200.0,-200.0,150.0,0.0,0.0,50.0,0.0,0.0,1.0)
context.camera('perspective', [('float fov', 50.0)])
context.worldBegin()

context.lightSource('sunsky', [
    ("vector sundir", (-0.5,-0.5,0.5)),
    ("float turbidity", 6.0),
    ])
context.shape('disk', [('float radius', 1000000.0)])

context.attributeBegin()
context.texture ("red", "color", "constant", [("color value", (1.0,0.0,0.0))])
context.material('matte', [("texture Kd", "red")])
context.shape('sphere', [("float radius", 2.0)])
context.attributeEnd()

context.worldEnd()

# do other stuff, or wait

print(context.printableStatistics(False))

print (len(context.framebuffer() ))

# end the render
context.exit()
context.cleanup()
del context


############################################


ctx = pylux.Context('one')
ctx.film('fleximage', [
    ('integer haltspp', 16),
    ])
ctx.sampler('metropolis', [])
ctx.surfaceIntegrator('path', [('string lightstrategy', 'one')])
ctx.renderer('hybrid', [])

ctx.lookAt(200.0,-200.0,150.0,0.0,0.0,50.0,0.0,0.0,1.0)
ctx.camera('perspective', [('float fov', 50.0)])
ctx.worldBegin()

ctx.lightSource('sunsky', [
    ("vector sundir", (-0.5,-0.5,0.5)),
    ("float turbidity", 6.0),
    ])
ctx.shape('disk', [('float radius', 1000000.0)])

ctx.attributeBegin()
ctx.texture ("red", "color", "constant", [("color value", (1.0,0.0,0.0))])
ctx.material('matte', [("texture Kd", "red")])
ctx.shape('sphere', [("float radius", 2.0)])
ctx.attributeEnd()
ctx.worldEnd()

while not ctx.statistics('sceneIsReady'):
    time.sleep(0.05)
ctx.addThread()
ctx.wait()

