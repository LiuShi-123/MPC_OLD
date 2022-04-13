#ifndef _REFWAY
#define _REFWAY
#include "Paras.h"
#include <cmath>
#include <string>
class _RefWay{
  private:
  void SetValue(std::string Chse, float sim_t);
  public:
  _Paras _prs;
  float Ref_X;
  float Ref_Y;
  float Ref_Phi;
  float Ref_V;
  float Ref_Delta;
  void RefValue(std::string Choose, float sim_t);
};
#endif
