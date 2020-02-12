#include <iostream>
#include <vector>

class Vel {
    public:
        double eci = 0;

        void Update(Rocket* rocket);
};

class Rocket {
    public:
        double t = 5.0;
        Vel vel;

        void Update();
};

void Vel::Update(Rocket* rocket) {
    std::cout << rocket->t << std::endl;
};

void Rocket::Update() {
    vel.Update(this);
};

int main()
{
    Rocket rocket;
    rocket.Update();
    return 0;    
};

