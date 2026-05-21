'use strict';
// app.js — Three.js visualizer + UI controller

// ── State ─────────────────────────────────────────────────────────────────
let robot      = makePlanarNDOF(3);
let mode       = 'fk';
let q          = [...robot.qHome];
let rightTab   = 'position';
let frameMode  = 'space';          // 'space' | 'body'  (Math tab Jacobian)
let modalFrame = 'space';          // frame of custom robot input
let ikTarget   = [robot.M[0][3] * 0.6, 0.5, 0];
let currentDof = 3;

// ── Three.js ──────────────────────────────────────────────────────────────
const canvas   = document.getElementById('three-canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.shadowMap.enabled = true;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d0f14);

let usePerspective = false;
let camera;

function makeCamera() {
  const w = canvas.clientWidth || 600, h = canvas.clientHeight || 400;
  if (usePerspective) {
    camera = new THREE.PerspectiveCamera(45, w / h, 0.01, 100);
    camera.position.set(0, 3, 5);
    camera.lookAt(0, 0, 0);
  } else {
    const span = 3.5, aspect = w / h;
    camera = new THREE.OrthographicCamera(
      -span * aspect, span * aspect, span, -span, -50, 50
    );
    camera.position.set(0, 0, 10);
    camera.lookAt(0, 0, 0);
  }
}
makeCamera();

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.1;

scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(3, 5, 3);
scene.add(dirLight);

const grid = new THREE.GridHelper(8, 16, 0x2a2f42, 0x1e2333);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

function makeAxisLine(dir, color) {
  const mat = new THREE.LineBasicMaterial({ color });
  const pts = [new THREE.Vector3(0,0,0), new THREE.Vector3(...dir)];
  return new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), mat);
}
scene.add(makeAxisLine([0.4,0,0], 0xf87171));
scene.add(makeAxisLine([0,0.4,0], 0x4ade80));
scene.add(makeAxisLine([0,0,0.4], 0x4f9cf9));

// ── Robot 3D objects ──────────────────────────────────────────────────────
const robotGroup  = new THREE.Group();
scene.add(robotGroup);
let linkMeshes  = [], jointMeshes = [], eeMesh = null, targetMesh = null;
const LINK_R = 0.035, JOINT_R = 0.065;

function buildRobotMeshes() {
  robotGroup.clear();
  linkMeshes = []; jointMeshes = [];
  const n = robot.dof;
  const colors = robot.linkColors.map(c => new THREE.Color(c));

  for (let i = 0; i < n; i++) {
    const geo = new THREE.CylinderGeometry(LINK_R, LINK_R, 1, 12);
    const mat = new THREE.MeshPhongMaterial({ color: colors[i % colors.length], shininess: 80 });
    const m   = new THREE.Mesh(geo, mat);
    robotGroup.add(m); linkMeshes.push(m);
  }
  for (let i = 0; i <= n; i++) {
    const geo = new THREE.SphereGeometry(JOINT_R, 16, 16);
    const c   = i === 0 ? 0x6b7494 : colors[(i-1) % colors.length];
    const mat = new THREE.MeshPhongMaterial({ color: c, shininess: 100 });
    const m   = new THREE.Mesh(geo, mat);
    robotGroup.add(m); jointMeshes.push(m);
  }

  // EE frame axes
  const eeGroup = new THREE.Group();
  [[1,0,0, 0xf87171],[0,1,0, 0x4ade80],[0,0,1, 0x4f9cf9]].forEach(([x,y,z,c]) => {
    const mat = new THREE.LineBasicMaterial({ color: c, linewidth: 2 });
    const pts = [new THREE.Vector3(0,0,0), new THREE.Vector3(x,y,z).multiplyScalar(0.22)];
    eeGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), mat));
  });
  robotGroup.add(eeGroup); eeMesh = eeGroup;

  if (targetMesh) scene.remove(targetMesh);
  const tgeo = new THREE.SphereGeometry(0.09, 12, 12);
  const tmat = new THREE.MeshPhongMaterial({ color: 0xf97b4f, transparent: true, opacity: 0.85 });
  targetMesh = new THREE.Mesh(tgeo, tmat);
  scene.add(targetMesh);

  updateRobotPose();
}

function updateRobotPose() {
  const n      = robot.dof;
  const frames = fkineJoints(robot.S, robot.Mlist, robot.M, q);
  // Skip frames[0] (base); use frames[1..n+1] → positions[0..n]
  const positions = frames.slice(1).map(T => new THREE.Vector3(T[0][3], T[1][3], T[2][3]));

  for (let i = 0; i <= n; i++) jointMeshes[i].position.copy(positions[i]);

  const eeT = frames[n + 1];
  eeMesh.position.set(eeT[0][3], eeT[1][3], eeT[2][3]);
  eeMesh.setRotationFromMatrix(new THREE.Matrix4().set(
    eeT[0][0], eeT[0][1], eeT[0][2], 0,
    eeT[1][0], eeT[1][1], eeT[1][2], 0,
    eeT[2][0], eeT[2][1], eeT[2][2], 0,
    0, 0, 0, 1
  ));

  // n links: positions[i] → positions[i+1]  (covers joint1→joint2, …, jointN→EE)
  for (let i = 0; i < n; i++) {
    const start = positions[i], end = positions[i + 1];
    const len   = start.distanceTo(end);
    if (len < 1e-4) { linkMeshes[i].visible = false; continue; }
    linkMeshes[i].visible = true;
    linkMeshes[i].position.copy(start.clone().add(end).multiplyScalar(0.5));
    linkMeshes[i].setRotationFromQuaternion(
      new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0,1,0), end.clone().sub(start).normalize())
    );
    linkMeshes[i].scale.set(1, len, 1);
  }

  if (targetMesh) {
    targetMesh.position.set(ikTarget[0], ikTarget[1], ikTarget[2]);
    targetMesh.visible = (mode === 'ik');
  }
}

// ── Resize ────────────────────────────────────────────────────────────────
function onResize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  renderer.setSize(w, h, false);
  if (usePerspective) {
    camera.aspect = w / h;
  } else {
    const span = 3.5, aspect = w / h;
    camera.left = -span*aspect; camera.right = span*aspect;
    camera.top  =  span;        camera.bottom = -span;
  }
  camera.updateProjectionMatrix();
}
new ResizeObserver(onResize).observe(canvas.parentElement);
onResize();

// ── Click-to-set IK target ────────────────────────────────────────────────
const raycaster = new THREE.Raycaster();
const planeZ    = new THREE.Plane(new THREE.Vector3(0,0,1), 0);

renderer.domElement.addEventListener('click', e => {
  if (mode !== 'ik') return;
  const rect = renderer.domElement.getBoundingClientRect();
  const ndc  = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width)  * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1
  );
  raycaster.setFromCamera(ndc, camera);
  const hit = new THREE.Vector3();
  if (raycaster.ray.intersectPlane(planeZ, hit)) {
    ikTarget = [hit.x, hit.y, 0];
    syncIKInputs();
    runIK();
  }
});

// ── Animation ─────────────────────────────────────────────────────────────
(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

// ── DOF picker ───────────────────────────────────────────────────────────
function buildDofPicker() {
  const el = document.getElementById('dof-picker');
  if (!el) return;
  el.innerHTML = '';
  [2,3,4,5,6].forEach(n => {
    const btn = document.createElement('button');
    btn.className = 'dof-btn' + (n === currentDof ? ' active' : '');
    btn.textContent = n;
    btn.addEventListener('click', () => {
      if (n === currentDof) return;
      const oldL = robot.linkLengths.slice();
      const oldMin = robot.qMin.slice();
      const oldMax = robot.qMax.slice();
      currentDof = n;
      const newL   = Array.from({length: n}, (_, i) => oldL[i]   || null);
      const newMin = Array.from({length: n}, (_, i) => oldMin[i] !== undefined ? oldMin[i] : -Math.PI);
      const newMax = Array.from({length: n}, (_, i) => oldMax[i] !== undefined ? oldMax[i] :  Math.PI);
      robot = makePlanarNDOF(n, newL);
      robot.qMin = newMin; robot.qMax = newMax;
      q = robot.qHome.map((qi, i) => Math.max(newMin[i], Math.min(newMax[i], qi)));
      ikTarget = [robot.M[0][3] * 0.6, 0.5, 0];
      updateVpBadge();
      buildDofPicker(); buildLinkEditors(); buildSliders(); buildRobotMeshes(); onQChanged();
    });
    el.appendChild(btn);
  });
}

function buildLinkEditors() {
  const el = document.getElementById('link-editors');
  if (!el) return;
  el.innerHTML = '';
  robot.linkLengths.forEach((L, i) => {
    const row = document.createElement('div');
    row.className = 'link-editor-row';
    row.innerHTML = `<label>L<sub>${i+1}</sub></label>
      <input type="number" class="link-len-input" data-idx="${i}"
        value="${L.toFixed(2)}" min="0.1" max="3" step="0.1">`;
    el.appendChild(row);
  });
}

function rebuildRobot() {
  const inputs  = document.querySelectorAll('.link-len-input');
  const lengths = Array.from(inputs).map(el => Math.max(0.1, parseFloat(el.value) || 1.0));
  const oldMin  = robot.qMin.slice();
  const oldMax  = robot.qMax.slice();
  robot = makePlanarNDOF(currentDof, lengths);
  robot.qMin = oldMin; robot.qMax = oldMax;
  q = [...robot.qHome];
  updateVpBadge(); buildSliders(); buildRobotMeshes(); onQChanged();
}

function updateVpBadge() {
  const el = document.getElementById('vp-badge');
  if (el) el.textContent = robot.name + ' · Product of Exponentials';
}

// ── Joint sliders + limit inputs ──────────────────────────────────────────
function buildSliders() {
  const container = document.getElementById('joint-sliders');
  if (!container) return;
  container.innerHTML = '';
  const n = robot.dof;

  for (let i = 0; i < n; i++) {
    const color  = robot.linkColors[i % robot.linkColors.length];
    const minDeg = Math.round(robot.qMin[i] * 180 / Math.PI);
    const maxDeg = Math.round(robot.qMax[i] * 180 / Math.PI);
    const degVal = (q[i] * 180 / Math.PI).toFixed(1);

    const div = document.createElement('div');
    div.className = 'joint-control';
    div.innerHTML = `
      <div class="joint-header">
        <span class="joint-label">
          <span class="joint-dot" style="background:${color}"></span>
          q<sub>${i+1}</sub>
        </span>
        <span class="joint-deg-display" id="jdeg-${i}">${degVal}°</span>
      </div>
      <div class="joint-row">
        <input type="number" class="limit-input" id="jmin-${i}"
          value="${minDeg}" step="5" title="Min angle (degrees)">
        <span class="limit-sep">°</span>
        <input type="range" id="js-${i}"
          min="${robot.qMin[i]}" max="${robot.qMax[i]}" step="0.001" value="${q[i]}">
        <span class="limit-sep">°</span>
        <input type="number" class="limit-input" id="jmax-${i}"
          value="${maxDeg}" step="5" title="Max angle (degrees)">
        <input type="number" id="jn-${i}" class="joint-num-input"
          value="${q[i].toFixed(3)}" step="0.01">
      </div>
    `;
    container.appendChild(div);

    const slider   = div.querySelector(`#js-${i}`);
    const numInput = div.querySelector(`#jn-${i}`);
    const minEl    = div.querySelector(`#jmin-${i}`);
    const maxEl    = div.querySelector(`#jmax-${i}`);
    const degEl    = div.querySelector(`#jdeg-${i}`);

    // Slider drag
    slider.addEventListener('input', () => {
      q[i] = parseFloat(slider.value);
      numInput.value = q[i].toFixed(3);
      degEl.textContent = (q[i] * 180 / Math.PI).toFixed(1) + '°';
      applyLimitFeedback(i);
      onQChanged();
    });

    // Typed radian value
    numInput.addEventListener('change', () => {
      let val = parseFloat(numInput.value) || 0;
      val = Math.max(robot.qMin[i], Math.min(robot.qMax[i], val));
      q[i] = val;
      numInput.value = val.toFixed(3);
      slider.value   = val;
      degEl.textContent = (val * 180 / Math.PI).toFixed(1) + '°';
      applyLimitFeedback(i);
      onQChanged();
    });
    numInput.addEventListener('keydown', e => { if (e.key === 'Enter') numInput.dispatchEvent(new Event('change')); });

    // Min limit
    minEl.addEventListener('change', () => {
      const minRad = (parseFloat(minEl.value) || -180) * Math.PI / 180;
      robot.qMin[i] = minRad;
      slider.min = minRad;
      if (q[i] < minRad) { q[i] = minRad; slider.value = minRad; numInput.value = minRad.toFixed(3); onQChanged(); }
      applyLimitFeedback(i);
    });

    // Max limit
    maxEl.addEventListener('change', () => {
      const maxRad = (parseFloat(maxEl.value) || 180) * Math.PI / 180;
      robot.qMax[i] = maxRad;
      slider.max = maxRad;
      if (q[i] > maxRad) { q[i] = maxRad; slider.value = maxRad; numInput.value = maxRad.toFixed(3); onQChanged(); }
      applyLimitFeedback(i);
    });

    applyLimitFeedback(i);
  }
}

// Color the slider thumb based on how close q[i] is to a limit
function applyLimitFeedback(i) {
  const slider = document.getElementById(`js-${i}`);
  if (!slider) return;
  const range = robot.qMax[i] - robot.qMin[i];
  const nearLimit = range > 0
    ? Math.min(q[i] - robot.qMin[i], robot.qMax[i] - q[i]) / range
    : 0;
  const color = nearLimit < 0.02 ? '#f87171'
              : nearLimit < 0.08 ? '#fbbf24'
              : 'var(--accent)';
  slider.style.setProperty('--thumb-color', color);
}

function syncSliders() {
  q.forEach((qi, i) => {
    const s   = document.getElementById(`js-${i}`);
    const num = document.getElementById(`jn-${i}`);
    const deg = document.getElementById(`jdeg-${i}`);
    if (s)   s.value = qi;
    if (num) num.value = qi.toFixed(3);
    if (deg) deg.textContent = (qi * 180 / Math.PI).toFixed(1) + '°';
    applyLimitFeedback(i);
  });
}

// ── Right-panel tabs ──────────────────────────────────────────────────────
document.querySelectorAll('.right-tab').forEach(btn => {
  btn.addEventListener('click', () => {
    rightTab = btn.dataset.tab;
    document.querySelectorAll('.right-tab').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    document.querySelectorAll('.tab-content').forEach(el => {
      el.classList.toggle('hidden', el.id !== `tab-${rightTab}`);
    });
    updateAllPanels();
  });
});

// ── Frame toggle (Math tab Jacobian) ──────────────────────────────────────
document.querySelectorAll('#frame-toggle .frame-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    frameMode = btn.dataset.frame;
    document.querySelectorAll('#frame-toggle .frame-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    updateMathTab();
  });
});

// ── Modal frame toggle ────────────────────────────────────────────────────
const FRAME_HINTS = {
  space: 'S: screw axes in world frame · FK = exp([S₁]q₁)·…·M',
  body:  'B: screw axes in EE frame    · FK = M·exp([B₁]q₁)·…'
};
document.querySelectorAll('#modal-frame-toggle .frame-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    modalFrame = btn.dataset.frame;
    document.querySelectorAll('#modal-frame-toggle .frame-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    const hint = document.getElementById('modal-frame-hint');
    if (hint) hint.textContent = FRAME_HINTS[modalFrame];
  });
});

// ── Panel updaters ────────────────────────────────────────────────────────
function updateFKPanel() {
  const T = fkine(robot.S, robot.M, q, 'space');
  ['x','y','z'].forEach((ax, i) => {
    const el = document.getElementById(`ee-${ax}`);
    if (el) el.textContent = T[i][3].toFixed(4);
  });
  const grid = document.getElementById('fk-matrix');
  if (grid) {
    grid.innerHTML = '';
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
      const cell = document.createElement('div');
      cell.className = 'matrix-cell' + (c === 3 && r < 3 ? ' highlight' : '');
      cell.textContent = T[r][c].toFixed(3);
      grid.appendChild(cell);
    }
  }
}

function updateJacobianPanel() {
  const Ja = jacoba(robot.S, robot.M, q);
  const n  = robot.dof;

  // Position tab — 3×n analytical Jacobian
  function fill3xn(headId, bodyId) {
    const head = document.getElementById(headId);
    const body = document.getElementById(bodyId);
    if (!head || !body) return;
    head.innerHTML = '<th></th>' + Array.from({length:n}, (_,c) => `<th>q${c+1}</th>`).join('');
    body.innerHTML = '';
    ['ẋ','ẏ','ż'].forEach((label, r) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `<th>${label}</th>` + Array.from({length:n}, (_,c) => `<td>${Ja[r][c].toFixed(4)}</td>`).join('');
      body.appendChild(tr);
    });
  }
  fill3xn('jac-head', 'jac-body');
}

function updateMathTab() {
  // Full 6×n Jacobian (space or body)
  const J6 = frameMode === 'space'
    ? spaceJacobian(robot.S, q)
    : bodyJacobian(robot.S, robot.M, q);
  const n   = robot.dof;
  const lbl = frameMode === 'space' ? 'Js' : 'Jb';
  const rowLabels = ['ω_x','ω_y','ω_z','v_x','v_y','v_z'];

  const head = document.getElementById('jac-head-math');
  const body = document.getElementById('jac-body-math');
  if (head && body) {
    head.innerHTML = `<th>${lbl}</th>` + Array.from({length:n}, (_,c) => `<th>q${c+1}</th>`).join('');
    body.innerHTML = '';
    rowLabels.forEach((label, r) => {
      const tr = document.createElement('tr');
      const isAngular = r < 3;
      tr.innerHTML = `<th style="color:${isAngular ? 'var(--accent2)' : 'var(--accent)'}">${label}</th>` +
        Array.from({length:n}, (_,c) => `<td>${J6[r][c].toFixed(4)}</td>`).join('');
      body.appendChild(tr);
    });
  }

  // Screw axes table — show S or B depending on frameMode
  const screwAxes = frameMode === 'space'
    ? robot.S
    : bodyJacobian(robot.S, robot.M, robot.qHome.map(() => 0)).map ? null : null; // computed below

  // For body axes: B_i = Adj(M^{-1}) * S_i  (at home config)
  let axes = robot.S;
  if (frameMode === 'body') {
    const AdM_inv = Adjoint(TransInv(robot.M));
    axes = zeros(6, n);
    for (let c = 0; c < n; c++) {
      const Si = Array.from({length:6}, (_, r) => robot.S[r][c]);
      const Bi = matVecMul(AdM_inv, Si);
      for (let r = 0; r < 6; r++) axes[r][c] = Bi[r];
    }
  }

  const desc = document.getElementById('screw-frame-desc');
  if (desc) desc.textContent = frameMode === 'space'
    ? 'Space-frame axes Sᵢ = [ω; v] — each column defines one joint\'s motion in the world frame'
    : 'Body-frame axes Bᵢ = [ω; v] — each column defines one joint\'s motion in the EE frame';

  const sh = document.getElementById('screw-head');
  const sb = document.getElementById('screw-body');
  if (sh && sb) {
    const axLbl = frameMode === 'space' ? 'S' : 'B';
    sh.innerHTML = `<th></th>` + Array.from({length:n}, (_,c) => `<th>${axLbl}<sub>${c+1}</sub></th>`).join('');
    sb.innerHTML = '';
    ['ω_x','ω_y','ω_z','v_x','v_y','v_z'].forEach((label, r) => {
      const tr = document.createElement('tr');
      const isAngular = r < 3;
      tr.innerHTML = `<th style="color:${isAngular ? 'var(--accent2)' : 'var(--accent)'}">${label}</th>` +
        Array.from({length:n}, (_,c) => `<td>${axes[r][c].toFixed(4)}</td>`).join('');
      sb.appendChild(tr);
    });
  }
}

function updateManipulabilityPanel() {
  const mu  = manipulability(robot.S, robot.M, q);
  const el  = document.getElementById('manip-value');
  const bar = document.getElementById('manip-bar');
  const ql  = document.getElementById('manip-quality');
  if (el)  el.textContent = mu.toFixed(3);
  if (bar) bar.style.width = Math.min(100, mu / 3 * 100) + '%';
  if (ql) {
    if      (mu < 0.05) { ql.textContent = 'Singularity';    ql.style.color = '#f87171'; }
    else if (mu < 0.5)  { ql.textContent = 'Limited';        ql.style.color = '#fbbf24'; }
    else if (mu < 1.5)  { ql.textContent = 'Good';           ql.style.color = '#4ade80'; }
    else                { ql.textContent = 'Excellent';       ql.style.color = '#4f9cf9'; }
  }
}

function updateDynamicsPanel() {
  try {
    const n    = robot.dof;
    const qd   = zeros(n), qdd = zeros(n);
    const tau  = fullTorques(robot, q, qd, qdd);
    const grav = gravityTorques(robot, q);
    const Mm   = massMatrix(robot, q);

    // Gravity torques (signed)
    const gEl = document.getElementById('gravity-bars');
    if (gEl) {
      const maxG = Math.max(1, ...grav.map(Math.abs));
      gEl.innerHTML = '';
      grav.forEach((gi, i) => {
        const pct = Math.abs(gi) / maxG * 100;
        const color = gi >= 0 ? '#4ade80' : '#f87171';
        const row = document.createElement('div');
        row.className = 'torque-row';
        row.innerHTML = `
          <span class="torque-label" style="color:${robot.linkColors[i]}">τ<sub>${i+1}</sub></span>
          <div class="torque-bar-track">
            <div class="torque-bar-fill" style="width:${pct.toFixed(1)}%;background:${color}"></div>
          </div>
          <span class="torque-val">
            <span style="color:${color};font-weight:700">${gi >= 0 ? '↺' : '↻'}</span>
            ${Math.abs(gi).toFixed(2)} N·m
          </span>`;
        gEl.appendChild(row);
      });
    }

    // Mass matrix
    const mmEl = document.getElementById('mass-matrix-display');
    if (mmEl) {
      if (n <= 4) {
        mmEl.className = 'mass-matrix-grid';
        mmEl.style.gridTemplateColumns = `auto repeat(${n}, 1fr)`;
        mmEl.innerHTML = '';
        const blank = document.createElement('div');
        blank.className = 'mass-cell header'; mmEl.appendChild(blank);
        for (let c = 0; c < n; c++) {
          const h = document.createElement('div');
          h.className = 'mass-cell header'; h.textContent = `q${c+1}`; mmEl.appendChild(h);
        }
        for (let r = 0; r < n; r++) {
          const rh = document.createElement('div');
          rh.className = 'mass-cell header'; rh.textContent = `q${r+1}`; mmEl.appendChild(rh);
          for (let c = 0; c < n; c++) {
            const cell = document.createElement('div');
            cell.className = 'mass-cell' + (r === c ? ' diag' : '');
            cell.textContent = Mm[r][c].toFixed(3); mmEl.appendChild(cell);
          }
        }
      } else {
        mmEl.className = ''; mmEl.innerHTML = '';
        const list = document.createElement('div'); list.className = 'torque-bars';
        const maxM = Math.max(...Array.from({length:n}, (_,i) => Mm[i][i]));
        for (let i = 0; i < n; i++) {
          const val = Mm[i][i], pct = val / maxM * 100;
          const row = document.createElement('div'); row.className = 'torque-row';
          row.innerHTML = `
            <span class="torque-label" style="color:${robot.linkColors[i]}">M<sub>${i+1}${i+1}</sub></span>
            <div class="torque-bar-track">
              <div class="torque-bar-fill" style="width:${pct.toFixed(1)}%;background:${robot.linkColors[i]}"></div>
            </div>
            <span class="torque-val">${val.toFixed(3)}</span>`;
          list.appendChild(row);
        }
        mmEl.appendChild(list);
      }
    }

    // Total torques
    const tEl = document.getElementById('torque-bars');
    if (tEl) {
      const maxT = Math.max(1, ...tau.map(Math.abs));
      tEl.innerHTML = '';
      tau.forEach((ti, i) => {
        const pct = Math.abs(ti) / maxT * 100;
        const color = robot.linkColors[i % robot.linkColors.length];
        const row = document.createElement('div'); row.className = 'torque-row';
        row.innerHTML = `
          <span class="torque-label" style="color:${color}">τ<sub>${i+1}</sub></span>
          <div class="torque-bar-track">
            <div class="torque-bar-fill" style="width:${pct.toFixed(1)}%;background:${color}"></div>
          </div>
          <span class="torque-val">${ti.toFixed(2)} N·m</span>`;
        tEl.appendChild(row);
      });
    }
  } catch(e) { /* degenerate config */ }
}

function updateAllPanels() {
  updateFKPanel();
  updateJacobianPanel();
  updateManipulabilityPanel();
  updateDynamicsPanel();
  if (rightTab === 'math') updateMathTab();
}

function onQChanged() {
  updateRobotPose();
  updateAllPanels();
}

// ── IK ────────────────────────────────────────────────────────────────────
function runIK() {
  const result = ikin(robot.S, robot.M, q, ikTarget, {
    tol: 1e-4, maxIter: 500, lambda: 0.05,
    qMin: robot.qMin, qMax: robot.qMax
  });
  q = result.q;
  syncSliders();
  onQChanged();

  const badge = document.getElementById('ik-status');
  if (badge) {
    badge.style.display = 'flex';
    badge.className = 'status-badge ' + (result.converged ? 'success' : 'warn');
    badge.textContent = result.converged
      ? `✓ Converged (${result.iters} iters)`
      : `⚠ Max iters (${result.iters})`;
  }
}

function syncIKInputs() {
  ['x','y','z'].forEach((ax, i) => {
    const el = document.getElementById(`ik-${ax}`);
    if (el) el.value = ikTarget[i].toFixed(3);
  });
}

['x','y','z'].forEach((ax, i) => {
  const el = document.getElementById(`ik-${ax}`);
  if (!el) return;
  el.addEventListener('change', () => { ikTarget[i] = parseFloat(el.value) || 0; runIK(); });
});
document.getElementById('btn-solve-ik')?.addEventListener('click', runIK);

// ── Mode tabs ─────────────────────────────────────────────────────────────
document.querySelectorAll('.mode-tab').forEach(tab => {
  tab.addEventListener('click', () => {
    mode = tab.dataset.mode;
    document.querySelectorAll('.mode-tab').forEach(t => t.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById('fk-controls')?.classList.toggle('hidden', mode !== 'fk');
    document.getElementById('ik-controls')?.classList.toggle('hidden', mode !== 'ik');
    if (targetMesh) targetMesh.visible = (mode === 'ik');
    document.getElementById('click-hint').style.opacity = mode === 'ik' ? '1' : '0';
    if (mode === 'ik') { syncIKInputs(); runIK(); }
  });
});

// ── Camera ────────────────────────────────────────────────────────────────
document.getElementById('btn-cam-toggle')?.addEventListener('click', () => {
  usePerspective = !usePerspective;
  makeCamera(); controls.object = camera; onResize();
  document.getElementById('btn-cam-toggle').textContent = usePerspective ? '⊙ Ortho' : '⊙ 3D';
});
document.getElementById('btn-reset-cam')?.addEventListener('click', () => {
  makeCamera(); controls.object = camera; onResize();
});

// ── Rebuild / Custom ──────────────────────────────────────────────────────
document.getElementById('btn-rebuild')?.addEventListener('click', rebuildRobot);

const modalOverlay = document.getElementById('modal-overlay');
document.getElementById('btn-custom-robot')?.addEventListener('click', () => modalOverlay?.classList.add('open'));
document.getElementById('btn-modal-cancel')?.addEventListener('click', () => modalOverlay?.classList.remove('open'));

document.getElementById('btn-modal-apply')?.addEventListener('click', () => {
  const screwText = document.getElementById('custom-screws')?.value ?? '';
  const mText     = document.getElementById('custom-m')?.value ?? '';
  const errEl     = document.getElementById('modal-error');
  try {
    const screwLines = screwText.trim().split('\n').filter(l => l.trim());
    // If body frame: convert B → S using S_i = Adj(M) * B_i
    let parsed = parseCustomRobot(screwLines, mText, modalFrame);
    robot = parsed;
    currentDof = robot.dof;
    q = [...robot.qHome];
    ikTarget = [robot.M[0][3] * 0.5, 0, 0];
    updateVpBadge();
    buildDofPicker(); buildLinkEditors(); buildSliders(); buildRobotMeshes(); onQChanged();
    modalOverlay?.classList.remove('open');
    if (errEl) errEl.textContent = '';
  } catch(e) {
    if (errEl) errEl.textContent = e.message;
  }
});

// ── Reset joints ──────────────────────────────────────────────────────────
document.getElementById('btn-reset-joints')?.addEventListener('click', () => {
  q = robot.qHome.map((qi, i) => Math.max(robot.qMin[i], Math.min(robot.qMax[i], qi)));
  syncSliders(); onQChanged();
});

// ── Init ──────────────────────────────────────────────────────────────────
buildDofPicker();
buildLinkEditors();
buildSliders();
buildRobotMeshes();
syncIKInputs();
onQChanged();
document.getElementById('click-hint').style.opacity = '0';
