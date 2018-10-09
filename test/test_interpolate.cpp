#include "../src/interpolate.h"


int main()
{
    Vector3d x = Vector3d(1,3,5);
    Vector3d y = Vector3d(2,6,10);
    Interp1d inter(x, y, "same");
    cout << inter(2) << endl;

    return 0;
}


