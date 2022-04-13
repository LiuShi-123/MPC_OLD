#include "RefWay.h"
void _RefWay::SetValue(std::string Chse, float sim_t)
{
	if (Chse == "Circle") {
		//_Circle
		_RefWay::Ref_X     = _RefWay::_prs.Ox + _RefWay::_prs.Or * sin(sim_t * _RefWay::_prs.Ov / _RefWay::_prs.Or);
		_RefWay::Ref_Y     = _RefWay::_prs.Oy - _RefWay::_prs.Or * cos(sim_t * _RefWay::_prs.Ov / _RefWay::_prs.Or);
	_RefWay::Ref_Phi   = sim_t * _RefWay::_prs.Ov / _RefWay::_prs.Or;
		_RefWay::Ref_V     = _RefWay::_prs.Ov;
		_RefWay::Ref_Delta = atan(_RefWay::_prs.Cl / _RefWay::_prs.Or);
	} else if (Chse == "Line") {
		//StrLine
		_RefWay::Ref_X     = _RefWay::_prs.Ov * sim_t;
                _RefWay::Ref_Y     = 0.;
                _RefWay::Ref_Phi   = 0.;
		_RefWay::Ref_V     = _RefWay::_prs.Ov;
		_RefWay::Ref_Delta = 0.;
	}
	//Else track path
}

void _RefWay::RefValue(std::string Choose, float sim_t){
  _RefWay::SetValue(Choose, sim_t);
}
