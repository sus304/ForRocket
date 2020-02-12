#include <iostream>
#include <vector>

class AClass {
    public:
        int value = 3;
};

class Factory {
    public:
        AClass Create() {
            AClass* a_obj = new AClass();
            return a_obj;
        };
};



int main()
{
    Factory factory;
    AClass a_obj = factory.Create();
    std::cout << a_obj.value << std::endl;
    return 0;    
};

