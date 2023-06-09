function j = JacobianFramePinholeOptReprojection(in1,in2,in3)
%JACOBIANFRAMEPINHOLEOPTREPROJECTION
%    [J1,J2] = JACOBIANFRAMEPINHOLEOPTREPROJECTION(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    31-Dec-2021 17:19:57

X = in3(:,1);
Y = in3(:,2);
Z = in3(:,3);
fx = in2(:,1);
fy = in2(:,2);
rx = in1(:,6);
ry = in1(:,5);
rz = in1(:,4);
t1 = in1(:,1);
t2 = in1(:,2);
t3 = in1(:,3);
t5 = cos(rx);
t6 = cos(ry);
t7 = cos(rz);
t8 = sin(rx);
t9 = sin(ry);
t10 = sin(rz);
t11 = X.*t6;
t12 = X.*t9;
t13 = t5.*t7;
t14 = t5.*t10;
t15 = t7.*t8;
t16 = t8.*t10;
t18 = Y.*t5.*t6;
t19 = Z.*t5.*t6;
t21 = Y.*t6.*t8;
t22 = Z.*t5.*t9;
t23 = Z.*t6.*t8;
t24 = Y.*t8.*t9;
t17 = t7.*t11;
t20 = t10.*t11;
t25 = -t12;
t26 = t9.*t14;
t27 = t9.*t15;
t28 = t9.*t16;
t29 = t9.*t13;
t30 = -t23;
t42 = t11+t22+t24;
t31 = -t26;
t32 = -t27;
t33 = t13+t28;
t34 = t16+t29;
t37 = t18+t30;
t45 = t3+t19+t21+t25;
t35 = Y.*t33;
t36 = Z.*t34;
t38 = t14+t32;
t39 = t15+t31;
t46 = 1.0./t45;
t40 = Y.*t38;
t41 = Z.*t39;
t47 = t46.^2;
t43 = -t40;
t44 = -t41;
t48 = t1+t17+t36+t43;

onesVec = ones(size(X));

j1 = [-fx.*t46,0.0.*onesVec,fx.*t47.*t48,fx.*t46.*(t20+t35+t44),-fx.*(t46.*(t7.*t25+Y.*t6.*t15+Z.*t6.*t13)+t42.*t47.*t48),-fx.*(t46.*(Y.*t34+Z.*t38)-t37.*t47.*t48)];
t49 = t2+t20+t35+t44;
j2 = [0.0.*onesVec,-fy.*t46,fy.*t47.*t49,-fy.*t46.*(t17+t36+t43),-fy.*(t46.*(t10.*t25+Y.*t6.*t16+Z.*t6.*t14)+t42.*t47.*t49),fy.*(t46.*(Y.*t39+Z.*t33)+t37.*t47.*t49)];

j = [j1;j2];

end