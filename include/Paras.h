#ifndef _PARAS
#define _PARAS
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <stdlib.h>
#include <string>
using namespace std;
class _Paras{
	public:
		YAML::Node pr = YAML::LoadFile("/home/ls/Project/CPlus/MPC/doc/Para.yaml");
		int Np = pr["Np"].as<int>();
		int Nc = pr["Nc"].as<int>();
		int Nx = pr["Nx"].as<int>();
		int Nu = pr["Nu"].as<int>();
		float Ox = pr["Ox"].as<float>();
		float Oy = pr["Oy"].as<float>();
		float Or = pr["Or"].as<float>();
		float Ov = pr["Ov"].as<float>();
		float Cl = pr["Cl"].as<float>();
		float Q = pr["Q"].as<float>();
		float R = pr["R"].as<float>();
                float Row = pr["Row"].as<float>();
                float DUmax0 = pr["DUmax0"].as<float>();
                float DUmax1 = pr["DUmax1"].as<float>();
                float Umax0 = pr["Umax0"].as<float>();
                float Umax1 = pr["Umax1"].as<float>();
                string Choose = pr["Choose"].as<string>();
                float Timee = pr["Timee"].as<float>();
                float T = pr["T"].as<float>();
};
#endif
