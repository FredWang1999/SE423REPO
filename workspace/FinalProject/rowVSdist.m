dis = [1    1.5 2   2.5 3 3.5 4 4.5 5 5.5];
row = [234  179 147 129 118 110 104 100 97 94];

p = polyfit(row,dis,3)
plot (row,dis,"o")
hold on
x = linspace(70,250,100)
plot(x,polyval(p,x))