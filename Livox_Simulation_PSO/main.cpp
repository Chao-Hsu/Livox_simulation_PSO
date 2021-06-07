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
		flag_cross_section = 1;
	}
	nMid70 = 2;
	nHorizon = 2;

	std::vector<float> tmp_para{

		//34.3 r=5
		//-4.7, 0.45, 0.0,  38.9913,  -21.9407,
		//-4.5, 0.30, 0.0, -12.25,    -44.68,
		//-4.5, 0.45, 0.0,  -4.02828, -39.6503, -26.65,
		//-4.3, 0.45, 0.0,   6.25617, -24.0357, -3.52407
		//modified
		//-4.7, 0.45, 0.0,  39.0, -22.0,
		//-4.5, 0.30, 0.0, -12.0, -45.0,
		//-4.5, 0.45, 0.0,  -0.0, -30.0, -27.0,
		//-4.3, 0.45, 0.0,   6.0, -24.0, -3.5

		////34.7 l=6
		//-4.12, 0.3, 0, 23.7001, -41.0582, 
		//-4.02, 0.42, 0, 5.20047, 1.24,
		//-4.22, 0.44, 0, -1.26155, -35.5 7, -45.6293, 
		//-4.08, 0.31, 0, 10.3892, -27.7128, 18.4967

		-4.55, 0.42, 0, 31.5011, 76.066, 
		-4.6, 0.43, 0, 19.6802, -42.8333, 
		-4.61, 0.37, 0, 9.72404, -32.7735, 13.1553, 
		-4.54, 0.42, 0, -6.06392, -15.9522, -82.4768
	};
	Mid70 tmp1(tmp_para.begin());
	Mid70 tmp2(tmp_para.begin() + 5);
	Horizon tmp3(tmp_para.begin() + 10);
	Horizon tmp4(tmp_para.begin() + 16);
	std::vector<Livox*> Sensors{ &tmp1,&tmp2,&tmp3,&tmp4 };

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

	/*float tmp_x;
	switch (flag_cross_section)
	{
	case choose_arch:
		tmp_x = Arch::l;
		break;
	case choose_cylinder:
		tmp_x = Cylinder::r;
		break;
	default:
		break;
	}
	float p_max[6]{ -(tmp_x - 0.7f), 0.45f, 0.0f,  90.0f,  90.0f,  90.0f };
	float p_min[6]{ -(tmp_x - 0.3f), 0.30f, 0.0f, -90.0f, -90.0f, -90.0f };
	PSO p;
	p.set_p_max(p_max);
	p.set_p_min(p_min);
	p.init();
	p.go();*/

	_CrtDumpMemoryLeaks();

	return 0;
}