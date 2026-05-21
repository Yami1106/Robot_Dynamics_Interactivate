# Robot_Dynamics_Interactivate[https://yamirobotdynamics.netlify.app]

## Overview

Every formula on this page is implemented in vanilla JavaScript with no robotics library dependencies. The math follows the **Modern Robotics** (Lynch & Park, 2017) convention throughout — screw axes, matrix exponentials, spatial inertia, and Recursive Newton-Euler — with all outputs verified analytically against known solutions.

---

## Kinematics

**Forward Kinematics — Product of Exponentials**

Each joint is described by a screw axis **S**ᵢ ∈ ℝ⁶ in the fixed world frame. The end-effector transform is:

```
T(q) = e^[S₁]q₁ · e^[S₂]q₂ · … · e^[Sₙ]qₙ · M
```

The matrix exponential is computed in closed form via Rodrigues' formula — no series truncation.

**Inverse Kinematics — Levenberg-Marquardt**

Position error **e** = p_target − p(q) is minimised with damped least-squares:

```
Δq = Jₐᵀ (Jₐ Jₐᵀ + λ²I)⁻¹ e
```

Joint limits are enforced by clamping after every iteration. Convergence threshold: ‖e‖ < 10⁻⁴ m.

**Jacobians**

| Frame | Formula |
|---|---|
| Space Jₛ | Column i = Ad(e^[S₁]q₁ ··· e^[Sᵢ₋₁]qᵢ₋₁) · Sᵢ |
| Body J_b | J_b = Ad(T⁻¹) · Jₛ |
| Analytical Jₐ | 3×n position sub-block of Jₛ |

Body-frame screw axes entered in the custom robot modal are converted at load time via **S = Ad(M) · B**.

---

## Dynamics

**Recursive Newton-Euler** runs two passes using 6×6 spatial inertia matrices G ∈ ℝ⁶ˣ⁶.

Forward pass — propagate spatial velocities and accelerations:
```
Vᵢ₊₁  = Ad(Tᵢ) Vᵢ + Aᵢ q̇ᵢ
V̇ᵢ₊₁  = Ad(Tᵢ) V̇ᵢ + Aᵢ q̈ᵢ + [Vᵢ₊₁, Aᵢ] q̇ᵢ
```

Backward pass — propagate spatial forces to joint torques:
```
Fᵢ = Ad(Tᵢ)ᵀ Fᵢ₊₁ + Gᵢ V̇ᵢ − adᵀ(Vᵢ) Gᵢ Vᵢ
τᵢ = Fᵢᵀ Aᵢ
```

The spatial inertia is placed at the **proximal joint frame** (not the COM), giving non-zero off-diagonal momentum-coupling blocks:

```
G_O = [ I_O      m[c]  ]     where c = [L/2, 0, 0]
      [ -m[c]    m I₃  ]
```

Verified on a 3-DOF robot at q = [0, 0, 0]: gravity torques **39.24 / 14.715 / 2.943 Nm** ✓ and mass matrix **symmetric** ✓.

| Quantity | Method |
|---|---|
| Gravity torques | RNE with q̇ = q̈ = 0 |
| Coriolis torques | RNE with q̈ = 0, g = 0 |
| Full inverse dynamics | RNE with all terms |
| Mass matrix (n×n) | Column-by-column via RNE with unit accelerations |
| Manipulability | μ = √det(JᵣJᵣᵀ), near-zero rows of Jₐ filtered to handle planar singularities |

---

## Features

| | |
|---|---|
| DOF picker | 2–6 joints; all math and visuals rebuild instantly |
| Joint controls | Slider + typed radian input with live degree display |
| Per-joint limits | Editable min°/max° per joint; thumb turns yellow (<8% margin) or red (<2%); IK respects limits |
| Frame toggle | Switch Jacobian and screw-axis display between space Jₛ and body J_b |
| IK click-to-target | Click viewport to place orange target sphere; LM solver runs on demand |
| Dynamics panel | Signed torque bar chart + full n×n mass matrix with diagonal highlighting |
| Custom robot | Paste screw axes and home transform M; space-frame or body-frame input |
| Camera | Orthographic (default) or perspective; full orbit/zoom/pan |

---

## Project Structure

```
├── index.html          # App shell and UI markup
├── netlify.toml        # Publish root + security headers
├── css/
│   └── style.css       # Design system — CSS custom properties, dark theme
└── js/
    ├── math.js         # Matrix / vector primitives, SO(3) and SE(3) exponentials, Adjoint, ad
    ├── kinematics.js   # fkine, fkineJoints, spaceJacobian, bodyJacobian, jacoba, ikin, manipulability
    ├── dynamics.js     # rne, gravityTorques, coriolisTorques, fullTorques, massMatrix
    ├── robots.js       # makePlanarNDOF, parseCustomRobot, rodSpatialInertia
    └── app.js          # Three.js scene, UI event handlers, render loop
```

No bundler, no transpiler, no npm. External CDN dependencies: **Three.js r134** (renderer + OrbitControls), **Inter** and **JetBrains Mono** (fonts).

---

## Implementation Notes

**Spatial inertia sign convention** — The off-diagonal blocks in G_O carry a specific sign from the adjoint transport formula G_O = Ad(T_CO)ᵀ · G_C · Ad(T_CO). Getting one sign wrong produces a non-symmetric mass matrix and wrong torques; verified by checking M₀₀ = 6.787 kg·m² against the analytic value.

**Rendering-correct FK** — `fkineJoints` returns n+2 frames (base + n joints + EE). Using `frames.slice(1)` gives n+1 positions so that exactly n links are drawn from positions[i] → positions[i+1]. Indexing from frames[0] instead causes the first link to be zero-length and the last link (joint_n → EE) to never render.

**Planar manipulability** — Planar robots always have a structural zero row in Jₐ (the out-of-plane component). Computing det(JₐJₐᵀ) directly gives 0. Rows with ‖row‖ < 10⁻⁶ are filtered before the determinant, recovering the correct in-plane measure.

**Body-frame IK** — All IK iteration runs in the space frame. Body-frame custom inputs are converted once at load time (S = Ad(M)·B) and stored as space axes, so the solver and Jacobian code are unchanged.

---

## Running Locally

```bash
# Python
python3 -m http.server 8080

# Node
npx serve .
```

Opening `index.html` via `file://` will fail — Three.js requires an HTTP server.

---

---

## References

- Lynch, K. M. & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.
- Murray, R. M., Li, Z. & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.
- Yoshikawa, T. (1985). Manipulability of robotic mechanisms. *The International Journal of Robotics Research*, 4(2), 3–9.
