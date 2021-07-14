#include "PSO.h"

unsigned main(int argc, char* argv[])
{
	if (argc >= 2)
	{
		THREADS = (unsigned)argv[1];
		flag_cross_section = (unsigned)argv[2];
	}
	else
	{
		THREADS = 12;
		flag_cross_section = choose_cylinder;
	}
	float tmp_x = (flag_cross_section == choose_arch) ? Arch::l : Cylinder::r;



	//nMid70 =2;
	//nHorizon = 2;
	//float p_max[6]{ -(tmp_x - 0.7f), 0.5f, 0.0f,  90.0f,  90.0f,  90.0f };
	//float p_min[6]{ -(tmp_x - 0.3f), 0.33f, 0.0f, -90.0f, -90.0f, -90.0f };
	//PSO p;
	//p.set_p_max(p_max);
	//p.set_p_min(p_min);
	//p.init();
	//p.go();

	////diff_amount
	//for (unsigned n = 1; n <= 5; n++)
	//{
	//	for (unsigned mid70 = 0; mid70 <= n; mid70++)
	//	{
	//		nMid70 = mid70;
	//		nHorizon = n - mid70;

	//		PSO p;
	//		p.set_p_max(p_max);
	//		p.set_p_min(p_min);
	//		p.init();
	//		p.go();
	//	}
	//}



	nMid70 = 2;
	nHorizon = 2;
	std::vector<float> tmp1_para{ -(tmp_x - 0.3f), 0.5, 0.0,  39.0, -22.0 };
	std::vector<float> tmp2_para{ -(tmp_x - 0.5f), 0.33, 0.0, -12.0, -45.0 };
	std::vector<float> tmp3_para{ -(tmp_x - 0.5f), 0.5, 0.0,  -0.0, -30.0, -27.0 };
	std::vector<float> tmp4_para{ -(tmp_x - 0.7f), 0.5, 0.0,   6.0, -24.0, -3.5 };
	std::vector<std::vector<float>> tmp_para{

		//34.3 r=5
		//-4.7, 0.45, 0.0,  38.9913,  -21.9407,
		//-4.5, 0.30, 0.0, -12.25,    -44.68,
		//-4.5, 0.45, 0.0,  -4.02828, -39.6503, -26.65,
		//-4.3, 0.45, 0.0,   6.25617, -24.0357, -3.52407
		//modified
		tmp1_para,
		tmp2_para,
		tmp3_para,
		tmp4_para

		////34.7 l=6
		//-4.12, 0.3, 0, 23.7001, -41.0582, 
		//-4.02, 0.42, 0, 5.20047, 1.24,
		//-4.22, 0.44, 0, -1.26155, -35.5 7, -45.6293, 
		//-4.08, 0.31, 0, 10.3892, -27.7128, 18.4967

		//-4.55, 0.42, 0, 31.5011, 76.066,
		//-4.6, 0.43, 0, 19.6802, -42.8333,
		//-4.61, 0.37, 0, 9.72404, -32.7735, 13.1553,
		//-4.54, 0.42, 0, -6.06392, -15.9522, -82.4768

		//-3.18, 0.31, 0, 43.9736, -43.87, 
		//-3.31, 0.38, 0, 35.9283, -3.61596, 
		//-3.19, 0.45, 0, -7.97672, -19.9178, -0.105686, 
		//-3.36, 0.32, 0, 10.4713, 25.15, 75.61

	};
	std::vector<Livox> Sensors;
	Sensors.reserve(nMid70 + nHorizon);
	for (size_t m = 0; m < nMid70; m++)
	{
		Mid70 tmp_mid70(tmp_para.at(m).begin());
		Sensors.emplace_back(tmp_mid70);
	}
	for (size_t h = 0; h < nHorizon; h++)
	{
		Horizon tmp_horizon(tmp_para.at(nMid70 + h).begin());
		Sensors.emplace_back(tmp_horizon);
	}

	switch (flag_cross_section)
	{
	case choose_arch:
		Arch::statistics(true);
		break;
	case choose_cylinder:
		Cylinder::statistics(true);
		break;
	default:
		break;
	}
	Livox::visualiation(Sensors);


	return 0;
}