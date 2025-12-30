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
        d = self.r * self.h
        y = self.v1 - target + self.v2 * self.h

        a0 = np.sqrt(d * d + 8 * self.r * np.abs(y))
        a = self.v2 + (1 if y > 0 else -1) * (a0 - d) / 2

        fh = -self.r * (a / d) - self.r * (1 if a > 0 else -1)

        self.v1 += self.v2 * self.h
        self.v2 += fh * self.h

    def LESO(self, actual_output):
        self.l1 = 3 * self.w0
        self.l2 = 3 * self.w0 * self.w0
        self.l3 = self.w0 * self.w0 * self.w0

        err = self.x1 - actual_output
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

    def update(self, target, actual_output):
        self.TD(target)
        self.LESO(actual_output)
        self.PD()


class SystemModel:
    def __init__(self, mass, damping, limit):
        self.position = 0
        self.velocity = 0
        self.mass = mass
        self.damping = damping
        self.limit = limit

    def update(self, control_input, dt):
        # 控制输入限幅
        if control_input > self.limit:
            control_input = self.limit
        if control_input < -self.limit:
            control_input = -self.limit

        # 物理方程: F = m*a, 考虑阻尼力
        acceleration = (control_input - self.damping * self.velocity) / self.mass

        # 欧拉积分
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        return self.position

if __name__ == '__main__':
    ladrc = LADRC(30.0, 1.0, 90.0, 0.005, 100.0)
    system = SystemModel(0.1, 0.1, 1000.0)
    target = 400
    actual_output = 0.0
    # 数据记录
    time_steps = 1000
    time = np.arange(time_steps) * ladrc.h
    targets = np.full(time_steps, target)
    outputs = np.zeros(time_steps)
    controls = np.zeros(time_steps)
    errors = np.zeros(time_steps)

    # 状态估计记录
    x1_estimates = np.zeros(time_steps)  # 位置估计
    x2_estimates = np.zeros(time_steps)  # 速度估计
    x3_estimates = np.zeros(time_steps)  # 扰动估计
    v1_tracks = np.zeros(time_steps)  # TD跟踪
    v2_tracks = np.zeros(time_steps)  # TD微分

    for i in range(time_steps):
        ladrc.update(target, actual_output)

        if ladrc.u > 1000:
            ladrc.u = 1000
        if ladrc.u < -1000:
            ladrc.u = -1000

        actual_output = system.update(ladrc.u, ladrc.h)

        outputs[i] = actual_output
        controls[i] = ladrc.u
        errors[i] = target - actual_output
        x1_estimates[i] = ladrc.x1
        x2_estimates[i] = ladrc.x2
        x3_estimates[i] = ladrc.x3
        v1_tracks[i] = ladrc.v1
        v2_tracks[i] = ladrc.v2

        if i % 100 == 0:
            print(
                f"Step: {i}, Target: {target:.1f}, Output: {actual_output:.2f}, Control: {ladrc.u:.2f}, Error: {errors[i]:.2f}")

    plt.figure(figsize=(8, 4))
    plt.plot(time, targets, 'b--', label='Target', linewidth=2)
    plt.plot(time, outputs, 'r-', label='Actual Output', linewidth=1.5)
    plt.plot(time, v1_tracks, 'g-', label='TD Tracking', linewidth=1, alpha=0.7)
    plt.ylabel('Position')
    plt.xlabel('Time (s)')
    plt.title('LADRC Tracking Performance')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Figure 2: Control output
    plt.figure(figsize=(8, 4))
    plt.plot(time, controls, 'purple', label='Control Output', linewidth=1.5)
    plt.ylabel('Control')
    plt.xlabel('Time (s)')
    plt.title('Control Output')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Figure 3: State estimation
    plt.figure(figsize=(5, 5))
    plt.plot(time, outputs, 'r-', label='Actual Position', linewidth=1.5, alpha=0.7)
    plt.plot(time, x1_estimates, 'g--', label='Position Estimate', linewidth=1.5)
    plt.plot(time, x2_estimates, 'b--', label='Velocity Estimate', linewidth=1.5)
    plt.ylabel('State')
    plt.xlabel('Time (s)')
    plt.title('State Estimation')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(8, 4))
    plt.plot(time, errors, 'r-', label='Tracking Error', linewidth=1.5)
    plt.plot(time, x3_estimates, 'orange', label='Disturbance Estimate', linewidth=1.5)
    plt.ylabel('Error/Disturbance')
    plt.xlabel('Time (s)')
    plt.title('Disturbance Estimate and Tracking Error')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    steady_state_error = np.mean(errors[-100:])
    max_overshoot = np.max(np.abs(outputs - target)) if np.max(outputs) > target else 0
    settling_time_idx = np.where(np.abs(errors) < 0.02 * target)[0]
    settling_time = settling_time_idx[0] * ladrc.h if len(settling_time_idx) > 0 else time[-1]

    print(f"\nPerformance Metrics:")
    print(f"Steady-state Error: {steady_state_error:.4f}")
    print(f"Maximum Overshoot: {max_overshoot:.4f}")
    print(f"Settling Time: {settling_time:.4f} seconds")
