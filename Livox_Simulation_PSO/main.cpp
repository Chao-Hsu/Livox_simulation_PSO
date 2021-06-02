#include "Livox.h"

unsigned main(int argc, char* argv[])
{
	THREADS = 12;
	flag_cross_section = 2;

	std::vector<float> tmp_para{

		//34.3 r=5
		-4.7, 0.45, 0.0,  38.9913,  -21.9407,
		-4.5, 0.30, 0.0, -12.25,    -44.68,
		-4.5, 0.45, 0.0,  -4.02828, -39.6503, -26.65,
		-4.3, 0.45, 0.0,   6.25617, -24.0357, -3.52407

		////34.7 l=6
		//-4.12, 0.3, 0, 23.7001, -41.0582, 
		//-4.02, 0.42, 0, 5.20047, 1.24,
		//-4.22, 0.44, 0, -1.26155, -35.57, -45.6293, 
		//-4.08, 0.31, 0, 10.3892, -27.7128, 18.4967

	};
	auto tmp_iter = tmp_para.begin();
	Mid70 tmp1(tmp_para.begin());
	Mid70 tmp2(tmp_para.begin() + 5);
	Horizon tmp3(tmp_para.begin() + 10);
	Horizon tmp4(tmp_para.begin() + 16);
	//Mid70 tmp1(tmp_para[0], tmp_para[1], tmp_para[2], tmp_para[3], tmp_para[4]);
	//Mid70 tmp2(tmp_para[5], tmp_para[6], tmp_para[7], tmp_para[8], tmp_para[9]);
	//Horizon tmp3(tmp_para[10], tmp_para[11], tmp_para[12], tmp_para[13], tmp_para[14], tmp_para[15]);
	//Horizon tmp4(tmp_para[16], tmp_para[17], tmp_para[18], tmp_para[19], tmp_para[20], tmp_para[21]);
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

	system("pause");

	return 0;
}