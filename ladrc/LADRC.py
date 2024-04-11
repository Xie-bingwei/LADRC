import numpy as np
import matplotlib.pyplot as plt

class LADRC:
    def __init__(self, wc, b0, w0, h, kp, kd, r):
        self.wc = wc
        self.b0 = b0
        self.w0 = w0
        # TD
        self.h = h
        self.v1 = 0
        self.v2 = 0
        # LESO
        self.l1 = 0
        self.l2 = 0
        self.l3 = 0
        self.x1 = 0
        self.x2 = 0
        self.x3 = 0
        # PD
        self.kp = kp
        self.kd = kd

        self.r = r

        self.u = 0.0

    def TD(self, input):
        fh = -input * input * (self.v1 - self.r) - 2 * input * self.v2
        self.v1 += self.v2 * self.h
        self.v2 += fh * self.h
        return None

    def LESO(self, feedback):
        self.l1 = 3 * self.w0
        self.l2 = 3 * self.w0 * self.w0
        self.l3 = self.w0 * self.w0 * self.w0

        err = self.x1 - feedback
        self.x1 += (self.x2 - self.l1 * err) * self.h
        self.x2 += (self.x3 - self.l2 * err + self.b0 * self.u) * self.h
        self.x3 += self.l3 * err * self.h

    def PD(self):
        self.kp = self.wc * self.wc
        self.kd = 2 * self.wc

        e1 = self.v1 - self.x1
        e2 = 0 - self.x2
        u0 = self.kp * e1 + self.kd * e2
        self.u = (u0 - self.x3) / self.b0
        return u0

if __name__ == '__main__':
    ladrc = LADRC(1.5, 1, 15, 0.005, 0, 0, 100)
    u0 = 0
    out = []
    i = 0
    target = 400
    for i in range(200):
        ladrc.TD(ladrc.r)
        ladrc.LESO(target)
        u0 = ladrc.PD()
        out = np.append(out, ladrc.x1)
        print("ADRC Output:", ladrc.x1)
      
    plt.plot(range(200), out, 'r')
    plt.axhline(y=target, color='b')
    plt.grid(True)
    plt.show()
