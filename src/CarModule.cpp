#include "CarModule.h"
void _CarModule::CarMove(float dt, float Cl, float Vk, float Deltak){
  _CarModule::Xkp1 = _CarModule::Xk + Vk * cos(_CarModule::Phik) * dt;
  _CarModule::Ykp1 = _CarModule::Yk + Vk * sin(_CarModule::Phik) * dt;
  _CarModule::Phikp1 = _CarModule::Phik + (Vk / Cl) * tan(Deltak) * dt;
  _CarModule::Xk = _CarModule::Xkp1;
  _CarModule::Yk = _CarModule::Ykp1;
  _CarModule::Phik = _CarModule::Phikp1;
}
