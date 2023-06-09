function [f,j] = LinescanPinholeOptReprojection(in1,in2,v)
%LinescanPinholeOptReprojection
%    [F,J1,J2] = LinescanPinholeOptReprojection(IN1,IN2,V)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    14-Jan-2023 00:25:35

K1 = in1(:,10);
K2 = in1(:,11);
T1 = in1(:,12);
X = in2(:,1);
Y = in2(:,2);
Z = in2(:,3);
fy = in1(:,8);
rw = in1(:,4);
rx = in1(:,5);
ry = in1(:,6);
rz = in1(:,7);
t1 = in1(:,1);
t2 = in1(:,2);
t3 = in1(:,3);
v0 = in1(:,9);

% if all([rw, rx, ry, rz,] == 0)
%     rw = 1;
% end
% 
% rw = [rw, rx, ry, rz];
% rxNew = rw./norm(rw);
% rw = rxNew(1);
% rx = rxNew(2);
% ry = rxNew(3);
% rz = rxNew(4);


t5 = rw.^2;
t6 = rx.^2;
t7 = ry.^2;
t8 = rz.^2;
t9 = X.*rw.*2.0;
t10 = X.*rx.*2.0;
t11 = X.*ry.*2.0;
t12 = X.*rz.*2.0;
t13 = Y.*rw.*2.0;
t14 = Y.*rx.*2.0;
t15 = Y.*ry.*2.0;
t16 = Y.*rz.*2.0;
t17 = Z.*rw.*2.0;
t18 = Z.*rx.*2.0;
t19 = Z.*ry.*2.0;
t20 = Z.*rz.*2.0;
t21 = rw.*t2.*2.0;
t22 = rw.*t3.*2.0;
t23 = rx.*t2.*2.0;
t24 = rx.*t3.*2.0;
t25 = ry.*t2.*2.0;
t26 = ry.*t3.*2.0;
t27 = rz.*t2.*2.0;
t28 = rz.*t3.*2.0;
t71 = X.*rw.*ry.*-2.0;
t72 = Y.*rw.*rz.*-2.0;
t73 = Z.*rw.*rx.*-2.0;
t29 = -t11;
t30 = -t14;
t31 = -t16;
t32 = -t17;
t33 = -t18;
t34 = -t26;
t35 = X.*t5;
t36 = X.*t6;
t37 = X.*t7;
t38 = X.*t8;
t39 = Y.*t5;
t40 = Y.*t6;
t41 = Y.*t7;
t42 = Y.*t8;
t43 = Z.*t5;
t44 = Z.*t6;
t45 = Z.*t7;
t46 = Z.*t8;
% t47 = ry.*t9;
t48 = rz.*t9;
t49 = ry.*t10;
t50 = rz.*t10;
t51 = rx.*t13;
% t52 = rz.*t13;
t53 = ry.*t14;
t54 = rz.*t15;
% t55 = rx.*t17;
t56 = ry.*t17;
t57 = rz.*t18;
t58 = rz.*t19;
t59 = t1.*t5;
t60 = t2.*t5;
t61 = t3.*t5;
t62 = t1.*t6;
t63 = t2.*t6;
t64 = t3.*t6;
t65 = t1.*t7;
t66 = t2.*t7;
t67 = t3.*t7;
t68 = t1.*t8;
t69 = t2.*t8;
t70 = t3.*t8;
t80 = t5+t6+t7+t8;
t81 = t10+t15+t20+t25;
t82 = t10+t15+t20+t28;
t74 = -t37;
t75 = -t38;
t76 = -t40;
t77 = -t42;
t78 = -t44;
t79 = -t45;
t83 = t14+t17+t22+t29;
t84 = t12+t13+t21+t33;
t85 = t12+t13+t24+t33;
t86 = t9+t19+t27+t31;
t87 = t11+t23+t30+t32;
t88 = t9+t19+t31+t34;
t89 = t35+t36+t53+t56+t57+t59+t62+t65+t68+t72+t74+t75;
t90 = t39+t41+t48+t49+t58+t60+t63+t66+t69+t73+t76+t77;
t91 = t43+t46+t50+t51+t54+t61+t64+t67+t70+t71+t78+t79;
t92 = t90.^2;
t93 = t90.^3;
t95 = 1.0./t91;
t94 = t92.^2;
t96 = t95.^2;
t97 = t95.^3;
t99 = t95.^5;
t98 = t96.^2;
t100 = K1.*t92.*t96;
t102 = T1.*t92.*t96.*3.0;
t101 = K2.*t94.*t98;
t103 = t100+t101+1.0;
t104 = t90.*t95.*t103;
f = [fy.*t89.*t95;v-v0-fy.*(t102+t104)];

onesVec = ones(size(X));


j1 = [fy.*t80.*t95,0.0.*onesVec,-fy.*t80.*t89.*t96,fy.*t95.*(t9+t19+t31+rw.*t1.*2.0)-fy.*t83.*t89.*t96,fy.*t95.*(t10+t15+t20+rx.*t1.*2.0)-fy.*t85.*t89.*t96,fy.*t95.*(t14+t17+t29+ry.*t1.*2.0)+fy.*t88.*t89.*t96,-fy.*t95.*(t12+t13+t33-rz.*t1.*2.0)-fy.*t82.*t89.*t96,t89.*t95,0.0.*onesVec,0.0.*onesVec,0.0.*onesVec,0.0.*onesVec];
mt1 = [0.0.*onesVec,-fy.*(t90.*t95.*(K1.*t80.*t90.*t96.*2.0+K2.*t80.*t93.*t98.*4.0)+t80.*t95.*t103+T1.*t80.*t90.*t96.*6.0),fy.*(t90.*t95.*(K1.*t80.*t92.*t97.*2.0+K2.*t80.*t94.*t99.*4.0)+T1.*t80.*t92.*t97.*6.0+t80.*t90.*t96.*t103),-fy.*(t84.*t95.*t103+t90.*t95.*(K1.*t84.*t90.*t96.*2.0-K1.*t83.*t92.*t97.*2.0+K2.*t84.*t93.*t98.*4.0-K2.*t83.*t94.*t99.*4.0)+T1.*t84.*t90.*t96.*6.0-T1.*t83.*t92.*t97.*6.0-t83.*t90.*t96.*t103),-fy.*(t87.*t95.*t103+t90.*t95.*(K1.*t87.*t90.*t96.*2.0-K1.*t85.*t92.*t97.*2.0-K2.*t85.*t94.*t99.*4.0+K2.*t87.*t93.*t98.*4.0)+T1.*t87.*t90.*t96.*6.0-T1.*t85.*t92.*t97.*6.0-t85.*t90.*t96.*t103)];
mt2 = [-fy.*(t81.*t95.*t103+t90.*t95.*(K1.*t81.*t90.*t96.*2.0+K2.*t81.*t93.*t98.*4.0+K1.*t88.*t92.*t97.*2.0+K2.*t88.*t94.*t99.*4.0)+T1.*t81.*t90.*t96.*6.0+T1.*t88.*t92.*t97.*6.0+t88.*t90.*t96.*t103),fy.*(-t86.*t95.*t103+t90.*t95.*(K1.*t82.*t92.*t97.*2.0-K1.*t86.*t90.*t96.*2.0+K2.*t82.*t94.*t99.*4.0-K2.*t86.*t93.*t98.*4.0)+T1.*t82.*t92.*t97.*6.0-T1.*t86.*t90.*t96.*6.0+t82.*t90.*t96.*t103),-t102-t104,-1.0.*onesVec,-fy.*t93.*t97,-fy.*t90.^5.*t99,fy.*t92.*t96.*-3.0];
j2 = [mt1,mt2];

j = [j1;j2];

end
