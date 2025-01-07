q1 = [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000];
q2= [90.000000, 45.511878, -66.337323, 0.000000, 110.825446, 90.000000];
q3= [90.000000, 39.955041, -0.065006, 0.000000, 50.109965, 90.000000];
q4= [90.000000, 45.511878, -66.337323, 0.000000, 110.825446, 90.000000];
q5= [0.000000, 24.692765, -33.754028, 0.000000, 99.061262, -0.000000];
q6= [0.000000, 19.517679, -4.344423, 0.000000, 74.826743, -0.000000];
q7= [0.000000, 24.692765, -33.754028, 0.000000, 99.061262, -0.000000];
q8= [-0.000000, -39.032357, 22.664723, 0.000000, 106.367634, -0.000000];
q9= [0.000000, 24.692765, -33.754028, 0.000000, 99.061262, -0.000000];
q10= [0.000000, 19.517679, -4.344423, 0.000000, 74.826743, -0.000000];
q11= [0.000000, 24.692765, -33.754028, 0.000000, 99.061262, -0.000000];
q12= [-90.000000, 45.511878, -66.337323, 0.000000, 110.825446, -90.000000];
q13= [-90.000000, 40.352335, 0.117669, 0.000000, 49.529996, -90.000000];
q14= [-90.000000, 45.511878, -66.337323, 0.000000, 110.825446, -90.000000];

q1 = deg2rad(q1);
q2 = deg2rad(q2);
q3 = deg2rad(q3);
q4 = deg2rad(q4);
q5 = deg2rad(q5);
q6 = deg2rad(q6);
q7 = deg2rad(q7);
q8 = deg2rad(q8);
q9 = deg2rad(q9);
q10 = deg2rad(q10);
q11 = deg2rad(q11);
q12 = deg2rad(q12);
q13 = deg2rad(q13);
q14 = deg2rad(q14);

t1=[0:.007:1.05]';
t2=[0:.007:2.84]';
t3=[0:.007:2.75]';
t4=[0:.007:0.95]';
t5=[0:.007:1.14]';
t6=[0:.007:1.14]';
t7=[0:.007:2.84]';
t8=[0:.007:2.84]';
t9=[0:.007:1.14]';
t10=[0:.007:1.14]';
t11=[0:.007:0.85]';
t12=[0:.007:2.84]';
t13=[0:.007:2.84]';
t14=[0:.007:0.92]';

%%
a=jtraj(q1,q2,t1);
figure()

subplot(2,1,1);
plot(t1,a(:,2));

subplot(2,1,2);
plot(t1,a(:,3));
%%
b=jtraj(q2,q3,t2);
figure()

subplot(2,1,1);
plot(t2,b(:,2));

subplot(2,1,2);
plot(t2,b(:,3));
%%
c=jtraj(q3,q4,t3);
figure()

subplot(2,1,1);
plot(t3,c(:,2));

subplot(2,1,2);
plot(t3,c(:,3));
%%
d=jtraj(q4,q5,t4);
figure()

subplot(2,1,1);
plot(t4,d(:,2));

subplot(2,1,2);
plot(t4,d(:,3));
%%
e=jtraj(q5,q6,t5);
figure()

subplot(2,1,1);
plot(t5,e(:,2));

subplot(2,1,2);
plot(t5,e(:,3));
%%
f=jtraj(q6,q7,t6);
figure()

subplot(2,1,1);
plot(t6,f(:,2));

subplot(2,1,2);
plot(t6,f(:,3));

%%
g=jtraj(q7,q8,t7);
figure()

subplot(2,1,1);
plot(t7,g(:,2));

subplot(2,1,2);
plot(t7,g(:,3));
%%
h=jtraj(q8,q1,t8);
figure()

subplot(2,1,1);
plot(t8,h(:,2));

subplot(2,1,2);
plot(t8,h(:,3));
%%
h=jtraj(q8,q9,t9);
figure()

subplot(2,1,1);
plot(t9,h(:,2));

subplot(2,1,2);
plot(t9,h(:,3));

%%
h=jtraj(q10,q11,t10);
figure()

subplot(2,1,1);
plot(t10,h(:,2));

subplot(2,1,2);
plot(t10,h(:,3));
%%
h=jtraj(q11,q12,t11);
figure()

subplot(2,1,1);
plot(t11,h(:,2));

subplot(2,1,2);
plot(t11,h(:,3));
%%
h=jtraj(q12,q13,t12);
figure()

subplot(2,1,1);
plot(t12,h(:,2));

subplot(2,1,2);
plot(t12,h(:,3));
%%
h=jtraj(q13,q14,t13);
figure()

subplot(2,1,1);
plot(t13,h(:,2));

subplot(2,1,2);
plot(t13,h(:,3));
%%
h=jtraj(q14,q1,t14);
figure()

subplot(2,1,1);
plot(t14,h(:,2));

subplot(2,1,2);
plot(t14,h(:,3));
