# Робастное управление с возмущениями
## 1. Теоретический анализ
### 1.1 Система
Манипулятор может быть описан уравнением:
$$
\begin{equation}\tag{1.1.1}
M(q)\ddot q + C(q, \dot q) \dot q + g(q) + D \dot q + F_c(\dot q) = u
\end{equation}
$$
$$
\begin{equation}\tag{1.1.2}
M(q)\ddot q + (C(q, \dot q) + D) \dot q + g(q) + F_c(\dot q) = u
\end{equation}
$$
где:
- $M(q)$ - неопределенная (для нее известна только примерная оценка) матрица инерции системы
- $C(q, \dot q)$ - неопределенная матрица центробежных и кориолисовых сил
- $g(q)$ - неопределенный вектор сил гравитации
- $D$ - неизвестная диагональная матрица вязкого трения
- $F_c$ - неизвестный вектор сил кулоновского трения
- $u$ - наше управление моментами (силами)
### 1.2 Линеаризация системы
Может быть проведена посредством использования управления в виде c использованием оценок параметров системы:
$$
\begin{equation}\tag{1.2.1}
u = \hat{M}(q) v + (\hat{C}(q, \dot q) + \hat D)\dot q + \hat g(q) + \hat F_c(\dot q),
\end{equation}
$$
где значения с шапкой - оценочные значения для реальных параметров системы, используемые для компенсации нелинейности системы, а $v$ - непосредственное управление.

Если поаставить такое управление в уравнение динамики, $\ddot q$ может быть выражено как:
$$
\begin{equation}\tag{1.2.2}
\ddot q = M^{-1}(q)\cdot (\widetilde{C}(q, \dot q)\dot q + \widetilde{D}\dot q + \widetilde{g}(q) + \widetilde{F_c}(\dot q)) +  M^{-1}(q) \hat M(q) v
\end{equation}
$$
Которое можно свернуть в:
$$
\begin{equation}\tag{1.2.3}
\ddot q = f(q, \dot q) +  B(q) v
\end{equation}
$$
где:
$$
\begin{equation}\tag{1.2.4}
f(q, \dot q) = M^{-1}(q)\cdot (\widetilde{C}(q, \dot q)\dot q + \widetilde{D}\dot q + \widetilde{g}(q) + \widetilde{F_c}(\dot q))
\end{equation}
$$
- $\widetilde{C} = C - \hat C$
- $\widetilde{D} = D - \hat D$
- $\widetilde{g} = g - \hat g$
- $\widetilde{F_c} = F_c - \hat F_c$
и
$$
\begin{equation}\tag{1.2.5}
B(q) v = M^{-1}(q) \hat M(q) v
\end{equation}
$$

### 1.3 Sliding surface
Задается уравнением:
$$
\begin{equation}\tag{1.3.1}
S = \dot {\widetilde {q}} + \lambda \widetilde {q},
\end{equation}
$$
где:
- $\widetilde {q} = q_{target} - q$ - ошибка по позиции суставов
- $\dot {\widetilde {q}} = \dot {q}_{target} - \dot q$ - ошибка по скорости суставов 
- $\lambda$ - матрица коэффициэнтов

Здесь, в качестве кандидата Ляпунова можно рассмотреть функцию:
$$
\begin{equation}\tag{1.3.2}
V = \frac{1}{2}\|S\|^2
\end{equation}
$$
Преобразуем функцию, подставив в нее выражение (1.2.3):
$$
\begin{equation}\tag{1.3.3}
\dot V = S^T \dot S = S^T (\ddot {\widetilde{q}} + \lambda \dot {\widetilde{q}}) = S^T (\ddot {q}_{target} - \ddot q + \lambda \dot {\widetilde{q}}) = S^T (\ddot {q}_{target} - f(q, \dot q) -  B(q) v + \lambda \dot {\widetilde{q}})
\end{equation}
$$
Нам нужно чтобы $\dot V < 0$, этого можно достичь, если:
$$
\begin{equation}\tag{1.3.4}
\ddot {q}_{target} - f(q, \dot q) -  B(q) v + \lambda \dot {\widetilde{q}} = -k\frac{S}{\|S\|}
\end{equation}
$$
Поскольку в этом случае:
$$
\begin{equation}\tag{1.3.5}
\dot V = S^T \dot S = S^T (\ddot {q}_{target} - f(q, \dot q) -  B(q) v + \lambda \dot {\widetilde{q}}) = -k\frac{S^T S}{\|S\|} = -k\frac{\|S\|^2}{\|S\|} = -k\|S\| < 0
\end{equation}
$$
Таким образом получаем, что система стабильна по Ляпунову, поскольку нашелся подходящий кандидат.

Преобразуя, получим:
$$
\begin{equation}\tag{1.3.6}
B(q) v = \ddot {q}_{target} - f(q, \dot q) + k\frac{S}{\|S\|} + \lambda \dot {\widetilde{q}}
\end{equation}
$$
Исходя из предроложения, что оценочные значения близки к истинным можно сказать, что значение $f(q, \dot q) \rightarrow 0$, а матрица $B(q) \rightarrow I$

Таким образом уравнение (1.3.6) упростится до:
$$
\begin{equation}\tag{1.3.7}
v = \ddot {q}_{target} + k\frac{S}{\|S\|} + \lambda \dot {\widetilde{q}}
\end{equation}
$$
Управление $v$ может быть разложено на две составляющие:
$$
\begin{equation}\tag{1.3.8}
v = v_n + v_s,
\end{equation}
$$
где:
- $v_n = \ddot q_{target} + \lambda \dot{\widetilde{q}}$ - номинальная составляющая управления
- $v_s = k\frac{S}{\|S\|}$ - составляющая управления для прихода на поверхность
### 1.4 Sliding condition
Чтобы система сходилась с определенной скоростью, можно ввести условие:
$$
\begin{equation}\tag{1.4.1}
\frac{1}{2}\frac{d}{dt}\|S\|^2 = S^T \dot S < -\eta \|S\|
\end{equation}
$$
Производная по времени от $S$ выглядит соответственно:
$$
\begin{equation}\tag{1.4.2}
\dot S = \ddot {\widetilde{q}} + \lambda \dot {\widetilde{q}} = v_n - \ddot q = v_n - f - B(v_n + v_s) = w - Bv_s
\end{equation}
$$
где:
$$
\begin{equation}\tag{1.4.3}
w = (I - B)v_n - f
\end{equation}
$$
Подставляя (1.4.2) в (1.4.1) получим:
$$
\begin{equation}\tag{1.4.4}
S^T (w - Bv_s) \leq \|S\|\|w\| - S^T B v_s \leq -\eta \|S\|
\end{equation}
$$
### 1.5 Робастный контроллер
Контроллер $v$ можно выбрать как:
$$
\begin{equation}\tag{1.5.1}
v_s = \frac{k}{\sigma_{max}}\hat M^{-1} \frac{S}{\|S\|} = \rho \frac{S}{\|S\|}
\end{equation}
$$
где $\sigma_{max}$ - максимальное сингулярное значение матрицы $M^{-1}$
Благодаря выбранному таким образом $v_s$:
$$
\tag{1.5.2}
\|S\|\|w\| - S^T B v_s \leq \|S\|\|w\| + \frac{k}{\lambda_{max}^2 \|S\|} S^T M^{-1}S \leq \|S\|\|w\| + k\|S\| < -\eta\|S\|
$$
Система будет сходиться при $k > \|w\| + \eta$
### 1.6 Итоговая система
$$
\begin{equation}
	\begin{cases}
		u = \hat{M}(q) v + (\hat{C}(q, \dot q) + \hat D)\dot q + \hat g(q) + \hat F_c(\dot q) \\
		v = \ddot {q}_{target} + \lambda \dot {\widetilde{q}} + v_s \\
		v_s = \frac{k}{\sigma_{max}}\hat M^{-1} \frac{S}{\|S\|} = \rho \frac{S}{\|S\|} \\
		S = \dot {\widetilde {q}} + \lambda \widetilde {q}
	\end{cases}.
\end{equation}
$$
## 2. Реализация и анализ эффективности
### 2.1 Модификация модели робота UR5
- Дополнительная масса на энд-эффекторе задается строкой:
``` python
sim.modify_body_properties("end_effector", mass=2)
```
- Коэффициэнты демпфирования задаются в виде numpy.array:
``` Python
D = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
```
	и устанавливаются с помощью:
```Python
sim.set_joint_damping(D)
```
- Кулоновское трение задается постоянным для каждого сустава:
```Python
F_c = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
```
	и устанавливается с помощью:
```Python
sim.set_joint_friction(F_c)
```
### 2.2 Реализация контроллеров
#### 2.2.1 Реализация PD контроллера
```Python
def get_worse_parameters(M, C, g, D, F_c):
    result_disp = 0.01
    result = ((result_disp * 2 * np.random.random(5) - result_disp) + 1)

    M_hat   = result[0] * M
    C_hat   = result[1] * C
    g_hat   = result[2] * g
    D_hat   = result[3] * D
    F_c_hat = result[4] * F_c

    return M_hat, C_hat, g_hat, D_hat, F_c_hat

  
  

def controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    t_history.append(t)
    joint_position_history.append(q)
    joint_velocity_history.append(dq)
    
    pin.computeAllTerms(model, data, q, dq)

    M_hat, C_hat, g_hat, D_hat, F_c_hat = get_worse_parameters(data.M, data.C, data.g, D, F_c)

    q_t = np.array([-0.5, -0.7, 1.0, 0.0, 0.0, 0.0], dtype=float)
    d_q_t = np.zeros(6, dtype=float)
    dd_q_t = np.zeros(6, dtype=float)

    q_err = q_t - q
    error_history.append(q_err)

    d_q_err = d_q_t - dq

    kp = 100
    kd = 20

    v = kp * q_err + kd * d_q_err + dd_q_t
    u = M_hat @ v + (C_hat + D_hat) @ dq + g_hat + F_c_hat
    u_history.append(u)

    return u
```

![[pd_errors 2.png]]
![[pd_joint_positions 2.png]]
![[pd_joint_velocities 2.png]]
![[pd_errors 3.png]]
![[pd_control 2.png]]
#### 2.2.2 Реализация робастного контроллера
```Python
def get_worse_parameters(M, C, g, D, F_c):
    result_disp = 0.01
    result = ((result_disp * 2 * np.random.random(5) - result_disp) + 1)

    M_hat   = result[0] * M
    C_hat   = result[1] * C
    g_hat   = result[2] * g
    D_hat   = result[3] * D
    F_c_hat = result[4] * F_c

    return M_hat, C_hat, g_hat, D_hat, F_c_hat

def V_s(M_hat, S):
    k = 1000
    epsilon = np.array([200, 200, 100, 100, 10, 100])
    sigma_max = 5

    S_norm = np.ones(6) * np.linalg.norm(S)

    S_norm = np.array(list([(eps * np.sign(S_norm[i]) if S_norm[i] <= eps else S_norm[i]) for i, eps in enumerate(epsilon)]))  

    rho = (k / sigma_max) * np.linalg.pinv(M_hat)

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
```
![[robust_joint_positions.png]]
![[robust_joint_velocities.png]]
![[robust_errors.png]]
![[robust_control.png]]
#### 2.2.3 Сравнение PD и Robust контроллеров
Видно, что в случае robust контроллера система сходится и остается в стабильном положении, в то время как PD не обеспечивает в данном случае даже зануления ошибки, а приводит к некоторому колебательному процессу
## Реализация пограничного слоя
Пограничный слой реализован в виде выражения:
$$
\begin{equation}
	v_s = 
	\begin{cases}
		\rho \dfrac{S}{\|S\|}, \space \|S\| > \epsilon \\
		\rho \dfrac{S}{\epsilon}, \space \|S\| \leq \epsilon
	\end{cases}
\end{equation}
$$
В коде это реализовано отдельно для каждого сустава и выглядит как:
```Python
def V_s(M_hat, S):
    k = 1000
    epsilon = np.array([200, 200, 100, 100, 10, 100])
    sigma_max = 5

    S_norm = np.ones(6) * np.linalg.norm(S)

    S_norm = np.array(list([(eps * np.sign(S_norm[i]) if S_norm[i] <= eps else S_norm[i]) for i, eps in enumerate(epsilon)]))  

    rho = (k / sigma_max) * np.linalg.pinv(M_hat)

    v_s = rho @ (S / np.mean(S_norm))

    return v_s
```

## Вывод
Robust Control значительно лучше работает в условиях, когда параметры системы известны лишь приблизительно.