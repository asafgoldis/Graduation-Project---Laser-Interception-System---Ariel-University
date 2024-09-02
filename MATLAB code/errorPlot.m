
font_size = 16;
L = [1.5:0.01:5];
d = 2.5;
x = d - L.*tan(atan(d./L)-pi/256);
x2 = d - L.*tan(atan(d./L)-pi/64);


figure
hold on
plot(L, x*100,'b')
plot(L(x==min(x)),min(x)*100, 'b--o')
text(L(x==min(x)),min(x)*100+1, {"min ("+L(x==min(x))+"[m], "+num2str(min(x)*100)+"[cm])"},'Color','blue','FontSize',font_size)
plot(L(x==max(x)),max(x)*100, 'b--o')
text(L(x==max(x))-1,max(x)*100+1, {"max ("+L(x==max(x))+"[m], "+num2str(max(x)*100)+"[cm])"},'Color','blue','FontSize',font_size)
text(3, 10, '8 bit resolution','Color','blue','FontSize',font_size)


plot(L, x2*100,'r')
plot(L(x2==min(x2)),min(x2)*100, 'r--o')
text(L(x2==min(x2)),min(x2)*100+1, {"min ("+L(x2==min(x2))+"[m], "+num2str(min(x2)*100)+"[cm])"},'Color','red','FontSize',font_size)
plot(L(x2==max(x2)),max(x2)*100, 'r--o')
text(L(x2==max(x2))-1,max(x2)*100-1, {"max ("+L(x2==max(x2))+"[m], "+num2str(max(x2)*100)+"[cm])"},'Color','red','FontSize',font_size)
text(3, 27, '6 bit resolution','Color','red','FontSize',font_size)

hold off
grid on
title('Step(L)','FontSize',20)
xlabel('L[m]','FontSize',20)
ylabel('Step[cm]','FontSize',20)