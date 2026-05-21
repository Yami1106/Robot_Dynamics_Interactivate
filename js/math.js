'use strict';
// math.js - Core matrix utilities (Product of Exponentials robotics)

function zeros(r, c) {
  if (c === undefined) return new Array(r).fill(0);
  return Array.from({length:r}, function(){return new Array(c).fill(0);});
}
function eye(n) {
  var I = zeros(n,n);
  for (var i=0;i<n;i++) I[i][i]=1;
  return I;
}
function matMul(A,B) {
  var m=A.length, k=B.length, n=B[0].length;
  var C=zeros(m,n);
  for (var i=0;i<m;i++) for (var p=0;p<k;p++) {
    if (A[i][p]===0) continue;
    for (var j=0;j<n;j++) C[i][j]+=A[i][p]*B[p][j];
  }
  return C;
}
function matAdd(A,B){return A.map(function(row,i){return row.map(function(v,j){return v+B[i][j];});});}
function matSub(A,B){return A.map(function(row,i){return row.map(function(v,j){return v-B[i][j];});});}
function matScale(A,s){return A.map(function(row){return row.map(function(v){return v*s;});});}
function transpose(A) {
  var m=A.length, n=A[0].length;
  return Array.from({length:n}, function(_,i){return Array.from({length:m},function(_,j){return A[j][i];});});
}
function vecNorm(v){return Math.sqrt(v.reduce(function(s,x){return s+x*x;},0));}
function vecAdd(a,b){return a.map(function(x,i){return x+b[i];});}
function vecSub(a,b){return a.map(function(x,i){return x-b[i];});}
function vecScale(v,s){return v.map(function(x){return x*s;});}
function matVecMul(A,v){return A.map(function(row){return row.reduce(function(s,aij,j){return s+aij*v[j];},0);});}
function NearZero(val){return Math.abs(val)<1e-6;}

function VecToso3(omg) {
  return [[0,-omg[2],omg[1]],[omg[2],0,-omg[0]],[-omg[1],omg[0],0]];
}
function so3ToVec(so3){return [so3[2][1],so3[0][2],so3[1][0]];}

function MatrixExp3(so3mat) {
  var omg=so3ToVec(so3mat), theta=vecNorm(omg);
  if (NearZero(theta)) return eye(3);
  var omghat=matScale(so3mat,1/theta), I=eye(3);
  return matAdd(matAdd(I,matScale(omghat,Math.sin(theta))),matScale(matMul(omghat,omghat),1-Math.cos(theta)));
}

function VecTose3(V) {
  var s=VecToso3([V[0],V[1],V[2]]);
  return [[s[0][0],s[0][1],s[0][2],V[3]],[s[1][0],s[1][1],s[1][2],V[4]],[s[2][0],s[2][1],s[2][2],V[5]],[0,0,0,0]];
}

function MatrixExp6(se3mat) {
  var omgtheta=so3ToVec([[se3mat[0][0],se3mat[0][1],se3mat[0][2]],[se3mat[1][0],se3mat[1][1],se3mat[1][2]],[se3mat[2][0],se3mat[2][1],se3mat[2][2]]]);
  var theta=vecNorm(omgtheta);
  if (NearZero(theta)) return [[1,0,0,se3mat[0][3]],[0,1,0,se3mat[1][3]],[0,0,1,se3mat[2][3]],[0,0,0,1]];
  var omgmat=[[se3mat[0][0]/theta,se3mat[0][1]/theta,se3mat[0][2]/theta],[se3mat[1][0]/theta,se3mat[1][1]/theta,se3mat[1][2]/theta],[se3mat[2][0]/theta,se3mat[2][1]/theta,se3mat[2][2]/theta]];
  var R=MatrixExp3(matScale(omgmat,theta));
  var st=Math.sin(theta),ct=Math.cos(theta),I=eye(3);
  var G=matAdd(matAdd(matScale(I,theta),matScale(omgmat,1-ct)),matScale(matMul(omgmat,omgmat),theta-st));
  var v=[se3mat[0][3]/theta,se3mat[1][3]/theta,se3mat[2][3]/theta];
  var Gv=matVecMul(G,v);
  return [[R[0][0],R[0][1],R[0][2],Gv[0]],[R[1][0],R[1][1],R[1][2],Gv[1]],[R[2][0],R[2][1],R[2][2],Gv[2]],[0,0,0,1]];
}

function TransInv(T) {
  var R=[[T[0][0],T[0][1],T[0][2]],[T[1][0],T[1][1],T[1][2]],[T[2][0],T[2][1],T[2][2]]];
  var p=[T[0][3],T[1][3],T[2][3]];
  var Rt=transpose(R), Rtp=vecScale(matVecMul(Rt,p),-1);
  return [[Rt[0][0],Rt[0][1],Rt[0][2],Rtp[0]],[Rt[1][0],Rt[1][1],Rt[1][2],Rtp[1]],[Rt[2][0],Rt[2][1],Rt[2][2],Rtp[2]],[0,0,0,1]];
}

function Adjoint(T) {
  var R=[[T[0][0],T[0][1],T[0][2]],[T[1][0],T[1][1],T[1][2]],[T[2][0],T[2][1],T[2][2]]];
  var p=[T[0][3],T[1][3],T[2][3]];
  var phat=VecToso3(p), pR=matMul(phat,R);
  var AdT=zeros(6,6);
  for (var i=0;i<3;i++) for (var j=0;j<3;j++) {
    AdT[i][j]=R[i][j]; AdT[i+3][j+3]=R[i][j]; AdT[i+3][j]=pR[i][j];
  }
  return AdT;
}

function ad(V) {
  var omgmat=VecToso3([V[0],V[1],V[2]]), vmat=VecToso3([V[3],V[4],V[5]]);
  var res=zeros(6,6);
  for (var i=0;i<3;i++) for (var j=0;j<3;j++) {
    res[i][j]=omgmat[i][j]; res[i+3][j+3]=omgmat[i][j]; res[i+3][j]=vmat[i][j];
  }
  return res;
}

function twist2ht(S,theta) {
  return MatrixExp6(VecTose3(vecScale(S,theta)));
}

function gaussianElim(A,b) {
  var n=b.length;
  var Aug=A.map(function(row,i){return row.concat([b[i]]);});
  for (var col=0;col<n;col++) {
    var maxRow=col;
    for (var r=col+1;r<n;r++) if (Math.abs(Aug[r][col])>Math.abs(Aug[maxRow][col])) maxRow=r;
    var tmp=Aug[col]; Aug[col]=Aug[maxRow]; Aug[maxRow]=tmp;
    if (Math.abs(Aug[col][col])<1e-14) continue;
    for (var r=0;r<n;r++) {
      if (r===col) continue;
      var f=Aug[r][col]/Aug[col][col];
      for (var j=col;j<=n;j++) Aug[r][j]-=f*Aug[col][j];
    }
  }
  return Aug.map(function(row,i){return row[n]/row[i];});
}

function solveLM(J,e,lambda) {
  var Jt=transpose(J), JtJ=matMul(Jt,J), n=JtJ.length;
  var A=JtJ.map(function(row,i){return row.map(function(v,j){return i===j?v+lambda*lambda:v;});});
  return gaussianElim(A,matVecMul(Jt,e));
}
