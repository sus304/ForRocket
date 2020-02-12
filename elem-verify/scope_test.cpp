#include <iostream>
#include <vector>

class Inner {
    public:
        double b;

        Inner() {}
        Inner(double t) {
            b = t;
        };
};

class MyClass {
    public:
        double a;
        Inner iin;

        MyClass() {};
        MyClass(double t) {
            a = t;
            iin = Inner(a);
        };
};


int main() {

    MyClass mc(2.0);
    std::cout << mc.iin.b << std::endl;

    return 0;    
};

