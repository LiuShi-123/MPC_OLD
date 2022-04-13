#ifndef _CARMODULE
#define _CARMODULE
#include <cmath>
class _CarModule{
  public:
    float Xk;
    float Yk;
    float Phik;
    float Xkp1;
    float Ykp1;
    float Phikp1;
    void CarMove(float dt, float Cl, float Vk, float Deltak);
};
#endif // !_CARMODULE
#define _CARMODULE
