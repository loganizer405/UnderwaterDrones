import rovpy as rovpy

rovpy.connectrov("STABILIZE", "udpin:0.0.0.0:14550")

rovpy.arm()
a = 0
while (a < 1000):
    a += 1
    rovpy.forward(0.5)
    rovpy.lateral(-0.5)

rovpy.disarm()
