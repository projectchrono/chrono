#-------------------------------------------------------------------------------
# Name:        demo_python_1
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


print ("First tutorial about Chrono::Engine in Python");


# Load the Chrono::Engine unit!!!
import ChronoEngine_PYTHON_core as chrono


# Test logging
chrono.GetLog().Bar()
chrono.GetLog() << "result is: " << 11+1.5 << "\n"
chrono.GetLog().Bar()


# Test vectors
my_vect1 = chrono.ChVectorD()
my_vect1.x=5
my_vect1.y=2
my_vect1.z=3
my_vect2 = chrono.ChVectorD(3,4,5)
my_vect4 = my_vect1*10 + my_vect2
my_len = my_vect4.Length()
print 'vect sum   =', my_vect1 + my_vect2
print 'vect cross =', my_vect1 % my_vect2
print 'vect dot   =', my_vect1 ^ my_vect2

# Test quaternions
my_quat = chrono.ChQuaternionD(1,2,3,4)
my_qconjugate = ~my_quat
print 'quat. conjugate  =', my_qconjugate
print 'quat. dot product=', my_qconjugate ^ my_quat
print 'quat. product=',     my_qconjugate % my_quat


# Print the class hierarchy of a chrono class
import inspect
inspect.getmro(chrono.ChStreamOutAsciiFile)

# Use matrices
my_matr = chrono.ChMatrixDynamicD(6,4)
my_matr[1,2]=4
my_matrb = chrono.ChMatrixDynamicD(4,6)
my_matrb.FillRandom(-2,2)
print my_matrb * 4


# Use the ChFunction classes
my_funct = chrono.ChFunction_Sine(0,0.5,3)
print 'function f(0.2)=', my_funct.Get_y(0.2)


# Inherit from the ChFunction, from the Python side,
# (do not forget the __init__ constructor)

class MySquareFunct (chrono.ChFunction):
    def __init__(self):
         chrono.ChFunction.__init__(self)
    def Get_y(self,x):
         return x*x

my_funct2 = MySquareFunct()
print 'function f(2) =', my_funct2.Get_y(2)
print 'function df/dx=', my_funct2.Get_y_dx(2)





