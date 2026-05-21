'use strict';
// kinematics.js - FK, IK, Jacobian. Depends on: math.js

function fkine(S, M, q, frame) {
  frame = frame || 'space';
  var n = q.length;
  if (frame === 'space') {
    var T = eye(4);
    for (var i = 0; i < n; i++)
      T = matMul(T, twist2ht(S.map(function(row){return row[i];}), q[i]));
    return matMul(T, M);
  } else {
    // Body frame: T = M * exp([B1]*q1) * ... * exp([Bn]*qn)
    var T = M.map(function(row){return row.slice();});
    for (var i = 0; i < n; i++)
      T = matMul(T, twist2ht(S.map(function(row){return row[i];}), q[i]));
    return T;
  }
}

// Returns array of (n+2) 4x4 world transforms:
//   [0]     = base frame (identity)
//   [1..n]  = world pose of each joint after applying screw motion + home offsets
//   [n+1]   = end-effector world pose
function fkineJoints(S, Mlist, M, q) {
  var n = q.length;
  // Accumulated home offsets: homeFrames[i] = Mlist[0]*...*Mlist[i]
  var homeFrames = [eye(4)];
  var Hacc = eye(4);
  for (var i = 0; i < Mlist.length; i++) {
    Hacc = matMul(Hacc, Mlist[i]);
    homeFrames.push(Hacc.map(function(row){return row.slice();}));
  }

  var frames = [eye(4)];            // frame[0] = base
  var Tacc = eye(4);
  for (var i = 0; i < n; i++) {
    var Si = S.map(function(row){return row[i];});
    Tacc = matMul(Tacc, twist2ht(Si, q[i]));
    frames.push(matMul(Tacc, homeFrames[i + 1]));
  }
  frames.push(matMul(Tacc, M));     // EE
  return frames;
}

// Space-frame Jacobian (6×n): each column is the space-frame screw axis of joint i
// after accounting for previous joint rotations.
function spaceJacobian(S, q) {
  var n = q.length;
  var Js = zeros(6, n);
  var T = eye(4);
  for (var i = 0; i < n; i++) {
    var Si = S.map(function(row){return row[i];});
    var col = (i === 0) ? Si : matVecMul(Adjoint(T), Si);
    for (var r = 0; r < 6; r++) Js[r][i] = col[r];
    T = matMul(T, twist2ht(Si, q[i]));
  }
  return Js;
}

// Body-frame Jacobian (6×n): Jb = Adj(T^{-1}) * Js
// Expresses EE twist in the body (EE) frame.
function bodyJacobian(S, M, q) {
  var Js = spaceJacobian(S, q);
  var T  = fkine(S, M, q, 'space');
  return matMul(Adjoint(TransInv(T)), Js);
}

// Analytical Jacobian (3×n): maps q̇ → ṗ (position-only, frame-independent)
function jacoba(S, M, q) {
  var n = q.length;
  var Js = spaceJacobian(S, q);
  var T = eye(4);
  for (var i = 0; i < n; i++) {
    var Si = S.map(function(row){return row[i];});
    T = matMul(T, twist2ht(Si, q[i]));
  }
  var Tfk = matMul(T, M);
  var p = [Tfk[0][3], Tfk[1][3], Tfk[2][3]];
  // sel maps [omega; v] of Js to linear velocity at p: ṗ = v + omega × p → sel = [-[p]|I] applied to Js
  var phat = [[0,-p[2],p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]];
  var I3 = eye(3);
  var sel = phat.map(function(row, i){
    return row.map(function(v){return -v;}).concat(I3[i]);
  });
  return matMul(sel, Js);
}

// Levenberg-Marquardt IK (position only).
// opts: { tol, maxIter, lambda, qMin[], qMax[] }
function ikin(S, M, q0, targetPos, opts) {
  opts = opts || {};
  var tol     = opts.tol     !== undefined ? opts.tol     : 1e-4;
  var maxIter = opts.maxIter !== undefined ? opts.maxIter : 1000;
  var lambda  = opts.lambda  !== undefined ? opts.lambda  : 0.05;
  var qMin    = opts.qMin || null;
  var qMax    = opts.qMax || null;
  var q = q0.slice();
  var n = q.length;
  for (var iter = 0; iter < maxIter; iter++) {
    var T = fkine(S, M, q, 'space');
    var p = [T[0][3], T[1][3], T[2][3]];
    var e = vecSub(targetPos, p);
    if (vecNorm(e) < tol) return {q:q, converged:true, iters:iter};
    var Ja = jacoba(S, M, q);
    var dq = solveLM(Ja, e, lambda);
    for (var i = 0; i < n; i++) {
      q[i] += dq[i];
      if (qMin !== null) q[i] = Math.max(qMin[i], q[i]);
      if (qMax !== null) q[i] = Math.min(qMax[i], q[i]);
    }
  }
  return {q:q, converged:false, iters:maxIter};
}

function det3(A) {
  return A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
       - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
       + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
}

// Manipulability μ = √det(Jr·JrT) where Jr filters near-zero rows of Ja
// (handles planar robots whose z-row is always 0).
function manipulability(S, M, q) {
  var Ja = jacoba(S, M, q);
  var m  = Ja.length, n = Ja[0].length;
  var activeRows = [];
  for (var r = 0; r < m; r++) {
    var rowNorm = 0;
    for (var c = 0; c < n; c++) rowNorm += Ja[r][c] * Ja[r][c];
    if (rowNorm > 1e-10) activeRows.push(r);
  }
  var k = activeRows.length;
  if (k === 0) return 0;
  var Jr  = activeRows.map(function(r){ return Ja[r]; });
  var JJT = matMul(Jr, transpose(Jr));
  var det = (k === 1) ? JJT[0][0]
           : (k === 2) ? JJT[0][0]*JJT[1][1] - JJT[0][1]*JJT[1][0]
           : det3(JJT);
  return Math.sqrt(Math.max(0, det));
}
