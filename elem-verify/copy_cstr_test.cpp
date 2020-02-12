#include <iostream>
#include <vector>

class MyClass {
    public:
        double a;

        MyClass(double t) {
            a = t;
        };
};

int main() {
    MyClass mc(2.0);

    std::cout << mc.a << std::endl;

    MyClass mc2 = mc;

    std::cout << mc.a << std::endl;
    std::cout << mc2.a << std::endl;

    mc2.a = 3.0;

    std::cout << mc.a << std::endl;
    std::cout << mc2.a << std::endl;


    return 0;    
};

