import numpy as np
import matplotlib.pyplot as plt

class LADRC:
    def __init__(self, wc, b0, w0, h, r):
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
        self.kp = 0
        self.kd = 0

        self.r = r
        self.u = 0.0
        self.u0 = 0.0

    def TD(self, target):
        fh = -self.r * self.r * (self.v1 - target) - 2 * self.r * self.v2
        self.v1 += self.v2 * self.h
        self.v2 += fh * self.h
        return None

    def LESO(self):
        self.l1 = 3 * self.w0
        self.l2 = 3 * self.w0 * self.w0
        self.l3 = self.w0 * self.w0 * self.w0

        err = self.x1 - self.u
        self.x1 += (self.x2 - self.l1 * err) * self.h
        self.x2 += (self.x3 - self.l2 * err + self.b0 * self.u) * self.h
        self.x3 += -self.l3 * err * self.h

    def PD(self):
        self.kp = self.wc * self.wc
        self.kd = 2 * self.wc

        e1 = self.v1 - self.x1
        e2 = 0 - self.x2
        self.u0 = self.kp * e1 + self.kd * e2
        self.u = (self.u0 - self.x3) / self.b0
        return None


out = []
out_hat = []
out_dt_hat = []
out_dt = []
out_f = []
if __name__ == '__main__':
    ladrc = LADRC(3.3, 1, 3.4, 0.005, 50)
    i = 0
    target = 400
    for i in range(800):
        ladrc.TD(target)
        ladrc.LESO()
        ladrc.PD()
        out = np.append(out, ladrc.u)
        out_hat = np.append(out_hat, ladrc.x1)
        out_dt_hat = np.append(out_dt_hat, ladrc.x2)
        out_dt = np.append(out_dt, ladrc.v2)
        out_f = np.append(out_f, ladrc.x3)
        print("ADRC Output:", ladrc.u, "TD_v1:", ladrc.x1)

    # ladrc = LADRC(0, 0, 0, 0.005, 1)
    # t = np.arange(0, 100, ladrc.h)
    # values = []
    # for t1 in t:
    #     ladrc.TD(target)
    #     values = np.append(values, ladrc.v1)
    fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(8, 6))

    axes[0].plot(range(800), out, 'r', label="y")
    axes[0].plot(range(800), out_hat, 'b', label="y_hat")
    axes[0].set_title('y and y_hat')

    axes[1].plot(range(800), out, 'r', label="y")
    axes[1].axhline(y=target, color='b', label="target")
    axes[1].set_title('target and y')

    axes[2].plot(range(800), out_dt_hat, 'b', label="dt_hat")
    axes[2].plot(range(800), out_dt, 'r', label="dt")
    axes[2].set_title('dt and dt_hat')

    axes[3].plot(range(800), out_f, 'r', label="f")
    axes[3].set_title('Extended State')

    # plt.plot(t, values, 'r')
    plt.show()
