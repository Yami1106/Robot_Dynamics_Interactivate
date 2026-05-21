'use strict';
// robots.js - Robot presets. Depends on: math.js

// Spatial inertia at frame O (proximal end of rod), rod extends along +x.
// G_O = [I_O,   m*[c]; -m*[c], m*I]   where c = [L/2, 0, 0]
function rodSpatialInertia(m, L) {
  var h = L / 2;
  var G = zeros(6, 6);
  G[0][0] = 1e-4;
  G[1][1] = m * L * L / 3;
  G[2][2] = m * L * L / 3;
  G[3][3] = m; G[4][4] = m; G[5][5] = m;
  // top-right  = m*[c]
  G[1][5] = -m * h;  G[2][4] =  m * h;
  // bottom-left = -m*[c]  (symmetric)
  G[4][2] =  m * h;  G[5][1] = -m * h;
  return G;
}

// Six-color palette: cycles for DOF > 6
var LINK_COLORS = ['#4f9cf9','#7c6af7','#f97b4f','#4ade80','#fbbf24','#f472b6'];

// Default link lengths for each DOF count
var DEFAULT_LENGTHS = {
  2: [1.2, 0.8],
  3: [1.0, 0.8, 0.6],
  4: [0.8, 0.65, 0.5, 0.35],
  5: [0.7, 0.55, 0.45, 0.35, 0.25],
  6: [0.6, 0.5, 0.4, 0.35, 0.3, 0.25]
};

// Build a planar N-DOF robot (all joints rotate about Z).
// lengths: optional array of link lengths; missing entries use defaults.
function makePlanarNDOF(n, lengths) {
  n = Math.max(1, Math.min(6, n || 3));
  var defL = DEFAULT_LENGTHS[n] || DEFAULT_LENGTHS[3];
  var L = [];
  for (var i = 0; i < n; i++) {
    var v = (lengths && lengths[i] != null) ? +lengths[i] : defL[i];
    L.push(Math.max(0.1, isNaN(v) ? defL[i] : v));
  }

  // Screw axes (space frame): S_i = [0,0,1, 0,-cumX_i, 0]
  var S = zeros(6, n);
  var cumX = 0;
  for (var i = 0; i < n; i++) {
    S[2][i] = 1;       // omega_z
    S[4][i] = -cumX;  // v_y = -joint_x (cross product rule)
    cumX += L[i];
  }

  // Home config: all links stretched along +x
  var totalL = L.reduce(function(a, b) { return a + b; }, 0);
  var M = [[1,0,0,totalL],[0,1,0,0],[0,0,1,0],[0,0,0,1]];

  function tx(d) { return [[1,0,0,d],[0,1,0,0],[0,0,1,0],[0,0,0,1]]; }
  var Mlist = [eye(4)];
  for (var i = 0; i < n; i++) { Mlist.push(tx(L[i])); }

  var Glist = L.map(function(li) { return rodSpatialInertia(1.0, li); });

  var colors = [];
  for (var i = 0; i < n; i++) { colors.push(LINK_COLORS[i % LINK_COLORS.length]); }

  return {
    name: n + '-DOF Planar',
    dof: n,
    S: S, M: M, Mlist: Mlist, Glist: Glist,
    g: [0, -9.81, 0],
    linkLengths: L,
    qHome: zeros(n),
    qMin: zeros(n).map(function() { return -Math.PI; }),
    qMax: zeros(n).map(function() { return  Math.PI; }),
    linkColors: colors
  };
}

// Kept for backward compatibility
function makePlanar3DOF(L1, L2, L3) {
  return makePlanarNDOF(3, [
    L1 !== undefined ? L1 : 1.0,
    L2 !== undefined ? L2 : 0.8,
    L3 !== undefined ? L3 : 0.6
  ]);
}

// Parse a custom robot from user-supplied screw axis text.
// frame: 'space' (default) or 'body'.
// If 'body', axes B_i are supplied; converted to space via S_i = Adj(M) * B_i.
function parseCustomRobot(screwLines, mLine, frame) {
  frame = frame || 'space';
  var n = screwLines.length;
  var S = zeros(6, n);
  for (var i = 0; i < n; i++) {
    var vals = screwLines[i].trim().split(/\s+/).map(Number);
    if (vals.length !== 6) { throw new Error('Screw axis ' + (i + 1) + ': need 6 values'); }
    for (var r = 0; r < 6; r++) { S[r][i] = vals[r]; }
  }
  var mv = mLine.trim().split(/\s+/).map(Number);
  if (mv.length !== 16) { throw new Error('M: need 16 values (row-major 4x4)'); }
  var M = [
    [mv[0],mv[1],mv[2],mv[3]],
    [mv[4],mv[5],mv[6],mv[7]],
    [mv[8],mv[9],mv[10],mv[11]],
    [mv[12],mv[13],mv[14],mv[15]]
  ];
  // Body-frame conversion: S_i = Adj(M) * B_i
  if (frame === 'body') {
    var AdM = Adjoint(M);
    var Sc = zeros(6, n);
    for (var i = 0; i < n; i++) {
      var Bi = [];
      for (var r = 0; r < 6; r++) { Bi.push(S[r][i]); }
      var Si = matVecMul(AdM, Bi);
      for (var r = 0; r < 6; r++) { Sc[r][i] = Si[r]; }
    }
    S = Sc;
  }
  function tr(p) { return [[1,0,0,p[0]],[0,1,0,p[1]],[0,0,1,p[2]],[0,0,0,1]]; }
  var Mlist = [eye(4)];
  var absPos = [0, 0, 0];
  for (var i = 0; i < n; i++) {
    var omega = [S[0][i], S[1][i], S[2][i]];
    var v     = [S[3][i], S[4][i], S[5][i]];
    var wn = vecNorm(omega);
    var jp = [0, 0, 0];
    if (wn > 1e-6) {
      var wx = omega[0] / wn, wy = omega[1] / wn, wz = omega[2] / wn;
      var vn = vecScale(v, 1 / wn);
      jp = [wy*vn[2] - wz*vn[1], wz*vn[0] - wx*vn[2], wx*vn[1] - wy*vn[0]];
    }
    Mlist.push(tr(vecSub(jp, absPos)));
    absPos = jp;
  }
  Mlist.push(tr(vecSub([M[0][3], M[1][3], M[2][3]], absPos)));
  var Glist = [];
  for (var i = 0; i < n; i++) {
    var G = zeros(6, 6);
    G[0][0] = 0.01; G[1][1] = 0.01; G[2][2] = 0.01;
    G[3][3] = 1;    G[4][4] = 1;    G[5][5] = 1;
    Glist.push(G);
  }
  var lc = [];
  for (var i = 0; i < n; i++) { lc.push(LINK_COLORS[i % LINK_COLORS.length]); }
  return {
    name: 'Custom', dof: n, S: S, M: M, Mlist: Mlist, Glist: Glist,
    g: [0, -9.81, 0],
    qHome: new Array(n).fill(0),
    qMin:  new Array(n).fill(-Math.PI),
    qMax:  new Array(n).fill( Math.PI),
    linkColors: lc
  };
}

var ROBOTS = { planar3: makePlanarNDOF(3) };
