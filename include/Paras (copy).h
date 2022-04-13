#ifndef _PARAS
#define _PARAS
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <stdlib.h>
class _Paras{
	public:
		YAML::Node pr = YAML::LoadFile("/home/ls/Project/CPlus/MPC/doc/Para.yaml");
		int Np = 60;//pr["Np"].as<int>();
		int Nc = 30;//pr["Nx"].as<int>();
		int Nx = 3;//pr["Nx"].as<int>();
		int Nu = 2;//pr["Nu"].as<int>();
		int Ox = 0;//pr["Ox"].as<int>();
		int Oy = 10;//pr["Oy"].as<int>();
		int Or = 8;//pr["Or"].as<int>();
		float Ov = 0.15;//pr["Ov"].as<float>();
		float Cd = 0.755;//pr["Cd"].as<float>();
		float Cl = 2.5772;//pr["Cl"].as<float>();
		float Wheelr = 0.317;//pr["Wheelr"].as<float>();
		float Q = 97.57;//pr["Q"].as<float>();
		float R = 48.93;//pr["R"].as<float>();
                float Row = 10;//pr["Row"].as<float>();
                float DUmax0 = 0.2;//pr["DUmax0"].as<float>();
                float DUmax1 = 0.436;//pr["DUmax1"].as<float>();
                float Umax0 = 0.05;//pr["Umax0"].as<float>();
                float Umax1 = 0.0082;//pr["Umax1"].as<float>();
};
#endif
