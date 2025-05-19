import sympy as sp
from sympy import sin, cos

x, y, z = sp.symbols('x y z')
th1, th2, th3 = sp.symbols('th1 th2 th3')
l1, l2 = sp.symbols('l1, l2')

x = l1 * cos(th1) + l2 * cos(th1+th2)
y = l1 * sin(th1) + l2 * sin(th1+th2)
z = th3

X = sp.Matrix([x, y, z])
J = X.jacobian([th1, th2, th3])
print(J)