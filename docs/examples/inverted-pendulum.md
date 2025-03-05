# Inverted Pendulum: System Modeling

The inverted pendulum is a classic example in control system literature due to its inherent instability when open loop. In this system, a pendulum is mounted on a motorized cart. The control objective is to balance the pendulum by applying a force to the cart.

---

## 1. Problem Setup and Design Requirements

- **System Description:**  
  A pendulum is mounted on a cart that can move horizontally. The system is unstable without control (the pendulum falls if not actively balanced).  
  **Inputs and Outputs:**
  - **Input:** Force \( F \) applied to the cart.
  - **Outputs:** Pendulum angle \( \theta \) and cart position \( x \).

- **Given Parameters:**
  - \( M \): Mass of the cart (0.5 kg)
  - \( m \): Mass of the pendulum (0.2 kg)
  - \( b \): Friction coefficient for the cart (0.1 N/m/s)
  - \( l \): Length to the pendulum's center of mass (0.3 m)
  - \( I \): Moment of inertia of the pendulum (0.006 kg·m²)

- **Design Criteria:**
  - **For single-input, single-output (SISO) control (pendulum angle control):**
    - Settling time for \( \theta \) less than 5 seconds.
    - After an impulse of 1 N·s, the pendulum deviates no more than 0.05 radians from vertical.
  - **For state-space design (SIMO) control:**
    - A 0.2 m step in cart position: settling time under 5 seconds and rise time under 0.5 seconds.
    - The pendulum angle deviation remains within 20° (0.35 radians) of the vertical.
    - Steady-state error less than 2% for both outputs.

---

## 2. Force Analysis and System Equations

### Free-Body Diagrams and Equations of Motion

1. **For the Cart (Horizontal Direction):**
   \[
   M\ddot{x} + b\dot{x} + N = F
   \]
   where \( N \) is the reaction force from the pendulum.

2. **For the Pendulum (Horizontal Reaction Force):**
   \[
   N = m\ddot{x} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta
   \]
   Substituting \( N \) into the cart equation gives:
   \[
   (M+m)\ddot{x} + b\dot{x} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta = F
   \]

3. **Pendulum's Rotational Equation (Moment about its Center of Mass):**
   \[
   (I+ml^2)\ddot{\theta} + mgl\sin\theta = -ml\ddot{x}\cos\theta
   \]

---

## 3. Linearization

Since the dynamics are nonlinear, the equations are linearized about the vertical equilibrium (\( \theta = \pi \)). Define the deviation:
\[
\phi = \theta - \pi
\]

**Small Angle Approximations:**
- \( \cos(\theta) = \cos(\pi+\phi) \approx -1 \)
- \( \sin(\theta) = \sin(\pi+\phi) \approx -\phi \)
- \( \dot{\theta}^2 \approx \dot{\phi}^2 \approx 0 \)

**Resulting Linearized Equations:**
1. **Angular Equation:**
   \[
   (I+ml^2)\ddot{\phi} - mgl\,\phi = ml\ddot{x}
   \]
2. **Cart Equation:**
   \[
   (M+m)\ddot{x} + b\dot{x} - ml\ddot{\phi} = u
   \]
   where \( u \) replaces \( F \) as the control input.

---

## 4. Transfer Function Derivation

Applying the Laplace transform (with zero initial conditions) to the linearized equations yields:

- **For the Pendulum Angle:**
  \[
  \frac{\Phi(s)}{U(s)} = \frac{\frac{ml}{q}\,s}{s^3 + \frac{b(I+ml^2)}{q}\,s^2 - \frac{(M+m)mgl}{q}\,s - \frac{bmgl}{q}}
  \]
  
- **Where:**
  \[
  q = \Big[(M+m)(I+ml^2) - (ml)^2\Big]
  \]

- **For the Cart Position:**
  \[
  P_{\text{cart}}(s) = \frac{X(s)}{U(s)} = \frac{ \frac{ (I+ml^2)s^2 - gml }{q} }{s^4 + \frac{b(I+ml^2)}{q}\,s^3 - \frac{(M+m)mgl}{q}\,s^2 - \frac{bmgl}{q}\,s}
  \]

---

## 5. State-Space Representation

The linearized system can be written in state-space form by defining the state vector:
\[
\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \phi \\ \dot{\phi} \end{bmatrix}
\]
and expressing the system as:
\[
\dot{\mathbf{x}} = A\mathbf{x} + B\,u,\quad \mathbf{y} = C\mathbf{x} + D\,u
\]

**Matrices:**

- **A Matrix:**
  ```matlab
  A = [  0,               1,              0,         0;
         0, -(I+ml^2)*b/p, (m^2*g*l^2)/p,      0;
         0,               0,              0,         1;
         0,   -m*l*b/p,   m*g*l*(M+m)/p,      0 ];
    ```
- **B Matrix:**
  ```matlab
  B = [ 0;
      (I+ml^2)/p;
       0;
      m*l/p ];
    ```
- **C Matrix:**
  ```matlab
  C = [1, 0, 0, 0;
     0, 0, 1, 0];
    ```

- **D Matrix:**
  ```matlab
  D = [0;
     0];
    ```