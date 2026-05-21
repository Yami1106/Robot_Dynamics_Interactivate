'use strict';
// dynamics.js - RNE inverse dynamics. Depends on: math.js

function rne(p) {
  var S=p.S, M=p.M, G=p.G, g=p.g;
  var q=p.jointPos, qd=p.jointVel, qdd=p.jointAcc, Ftip=p.Ftip;
  var n=q.length;
  var Vframes  = Array.from({length:n+1}, function(){return zeros(6);});
  var Vdframes = Array.from({length:n+1}, function(){return zeros(6);});
  var A   = Array.from({length:n},   function(){return zeros(6);});
  var AdTi= Array.from({length:n+1}, function(){return zeros(6,6);});

  Vframes[0]  = zeros(6);
  Vdframes[0] = [0,0,0,-g[0],-g[1],-g[2]];

  var Mi = eye(4);
  for (var i=0;i<n;i++) {
    Mi = matMul(Mi, M[i]);
    var Si = S.map(function(row){return row[i];});
    A[i] = matVecMul(Adjoint(TransInv(Mi)), Si);
    var Ti = matMul(MatrixExp6(VecTose3(vecScale(A[i],-q[i]))), TransInv(M[i]));
    AdTi[i] = Adjoint(Ti);
    Vframes[i+1]  = vecAdd(matVecMul(AdTi[i], Vframes[i]),  vecScale(A[i], qd[i]));
    var adV_A_qd  = matVecMul(ad(Vframes[i+1]), vecScale(A[i], qd[i]));
    Vdframes[i+1] = vecAdd(vecAdd(matVecMul(AdTi[i], Vdframes[i]), vecScale(A[i], qdd[i])), adV_A_qd);
  }
  AdTi[n] = Adjoint(TransInv(M[n]));

  var Fi = Ftip.slice();
  var tau = zeros(n);
  for (var i=n-1;i>=0;i--) {
    var GVdot = matVecMul(G[i], Vdframes[i+1]);
    var adTV  = matVecMul(transpose(ad(Vframes[i+1])), matVecMul(G[i], Vframes[i+1]));
    var AdTt  = transpose(AdTi[i+1]);
    Fi = vecAdd(vecSub(matVecMul(AdTt, Fi), adTV), GVdot);
    tau[i] = Fi.reduce(function(s,fi,r){return s+fi*A[i][r];}, 0);
  }
  return {tau:tau, V:Vframes, Vdot:Vdframes};
}

function gravityTorques(robot, q) {
  var n=q.length;
  return rne({S:robot.S,M:robot.Mlist,G:robot.Glist,
    jointPos:q,jointVel:zeros(n),jointAcc:zeros(n),g:robot.g,Ftip:zeros(6)}).tau;
}

function coriolisTorques(robot, q, qd) {
  var n=q.length;
  return rne({S:robot.S,M:robot.Mlist,G:robot.Glist,
    jointPos:q,jointVel:qd,jointAcc:zeros(n),g:[0,0,0],Ftip:zeros(6)}).tau;
}

function fullTorques(robot, q, qd, qdd) {
  return rne({S:robot.S,M:robot.Mlist,G:robot.Glist,
    jointPos:q,jointVel:qd,jointAcc:qdd,g:robot.g,Ftip:zeros(6)}).tau;
}

function massMatrix(robot, q) {
  var n=q.length;
  var Mmat=zeros(n,n);
  for (var i=0;i<n;i++) {
    var acc=zeros(n); acc[i]=1;
    var tau=rne({S:robot.S,M:robot.Mlist,G:robot.Glist,
      jointPos:q,jointVel:zeros(n),jointAcc:acc,g:[0,0,0],Ftip:zeros(6)}).tau;
    for (var j=0;j<n;j++) Mmat[j][i]=tau[j];
  }
  return Mmat;
}
