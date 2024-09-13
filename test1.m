load envs.mat

sp = envs{1};
sp.start_conf = [...
      0	0	50
0	0	175
0	0	175
0	0	175
0	0	100
];


sp.goal_conf = [...
       0	0	50
-45	0	175
45	0	175
0	-10	175
45	-15	100];

sol = searchAlgorithm(sp, 50, false);
p = pathConversion2(sol.path);
animate(sp, p);

