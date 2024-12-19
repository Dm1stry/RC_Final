import numpy as np
from simulator import Simulator
from pathlib import Path
import os
import pinocchio as pin
from typing import Dict
import matplotlib.pyplot as plt

RANDOM_SEED = 42
np.random.seed(RANDOM_SEED)

model = pin.buildModelFromMJCF("robots/universal_robots_ur5e/ur5e.xml")
data = model.createData()
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir, "robots/universal_robots_ur5e/ur5e.xml")


t_history = list()
joint_position_history = list()
joint_velocity_history = list()
error_history = list()
u_history = list()

D = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
F_c = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

def get_worse_parameters(M, C, g, D, F_c):
    result_disp = 0.01
    result = ((result_disp * 2 * np.random.random(5) - result_disp) + 1)

    M_hat   = result[0] * M
    C_hat   = result[1] * C
    g_hat   = result[2] * g
    D_hat   = result[3] * D
    F_c_hat = result[4] * F_c

    return M_hat, C_hat, g_hat, D_hat, F_c_hat
	

integral = 0

def V_s(M_hat, S):
    k = 1000
    epsilon = np.array([200, 200, 100, 100, 10, 100])
    sigma_max = 5

    S_norm = np.ones(6) * np.linalg.norm(S)

    S_norm = np.array(list([(eps * np.sign(S_norm[i]) if S_norm[i] <= eps else S_norm[i]) for i, eps in enumerate(epsilon)]))

    rho = (k / sigma_max) * np.linalg.pinv(M_hat) 

    integral += S / np.mean(S_norm)

    v_s = rho @ (S / np.mean(S_norm))
    
    return v_s


def controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    t_history.append(t)
    joint_position_history.append(q)
    joint_velocity_history.append(dq)
    
    pin.computeAllTerms(model, data, q, dq)

    M_hat, C_hat, g_hat, D_hat, F_c_hat = get_worse_parameters(data.M, data.C, data.g, D, F_c)

    q_t = np.array([-0.5, -0.7, 1.0, 0.0, 0.0, 0.0], dtype=float)
    d_q_t = np.zeros(6, dtype=float)
    dd_q_t = np.zeros(6, dtype=float)

    lambdas = np.array([400, 400, 400, 100, 100, 10], dtype=float)

    q_err = q_t - q
    error_history.append(q_err)

    d_q_err = d_q_t - dq

    S = d_q_err + lambdas * q_err

    V = dd_q_t + lambdas * d_q_err + V_s(M_hat, S)

    u = M_hat @ V + (C_hat + D_hat) @ dq + g_hat + F_c_hat * np.sign(dq)

    u_history.append(u)

    return u


def plot_results():
    """Plot and save simulation results."""
    global t_history
    global joint_position_history
    global joint_velocity_history
    global error_history
    global u_history
    
    t_history = np.array(t_history)
    joint_position_history = np.array(joint_position_history)
    joint_velocity_history = np.array(joint_velocity_history)
    error_history = np.array(error_history)
    u_history = np.array(u_history)

    # Joint joint_position_history plot
    plt.figure(figsize=(10, 6))
    for i in range(joint_position_history.shape[1]):
        plt.plot(t_history, joint_position_history[:, i], label=f'Joint {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Positions [rad]')
    plt.title('Joint Positions over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig('log/plots/joint_positions.png')
    plt.close()
    
    # Joint velocities plot
    plt.figure(figsize=(10, 6))
    for i in range(joint_velocity_history.shape[1]):
        plt.plot(t_history, joint_velocity_history[:, i], label=f'Joint {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Velocities [rad/s]')
    plt.title('Joint Velocities over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig('log/plots/joint_velocities.png')
    plt.close()
	
    # Error plot
    plt.figure(figsize=(10, 6))
    for i in range(error_history.shape[1]):
        plt.plot(t_history, error_history[:, i], label=f'Joint {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Velocities [rad/s]')
    plt.title('Joint Velocities over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig('log/plots/errors.png')
    plt.close()
	
    # Control plot
    plt.figure(figsize=(10, 6))
    for i in range(u_history.shape[1]):
        plt.plot(t_history, u_history[:, i], label=f'Joint {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Velocities [rad/s]')
    plt.title('Joint Velocities over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig('log/plots/control.png')
    plt.close()

def main():
    # Create logging directories
    Path("log/videos").mkdir(parents=True, exist_ok=True)
    
    print("\nRunning task space controller...")
    sim = Simulator(
        xml_path="robots/universal_robots_ur5e/scene.xml",
        enable_task_space=False,
        show_viewer=True,
        record_video=True,
        video_path="log/videos/final.mp4",
        fps=30,
        width=1920,
        height=1080
    )
    sim.set_joint_damping(D)
    sim.set_joint_friction(F_c)
    sim.modify_body_properties("end_effector", mass=2)
    sim.set_controller(controller)
    sim.run(time_limit=30.0)

main()
plot_results()