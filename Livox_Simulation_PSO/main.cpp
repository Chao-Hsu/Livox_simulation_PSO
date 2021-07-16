#define _WIN32_WINNT 0x0500
#include "PSO.h"
typedef struct
{
	int m1;
	int m2;
	int h1;
	int h2;
	float tx;
	float ty;
	float tz;
	float rx;
	float ry;
	float rz;
	float nCompleted;
	float percentage;
}output;

int main(int argc, char* argv[])
{
	HWND hWnd = GetConsoleWindow();
	ShowWindow(hWnd, 1);

	if (argc >= 2)
	{
		THREADS = (unsigned)argv[1];
		flag_cross_section = (unsigned)argv[2];
	}
	else
	{
		THREADS = 8;
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
	//std::vector<std::vector<float>> tmp_para{
	//	tmp1_para,
	//	tmp2_para,
	//	tmp3_para,
	//	tmp4_para

	//	//34.3 r=5
	//	//-4.7, 0.45, 0.0,  38.9913,  -21.9407,
	//	//-4.5, 0.30, 0.0, -12.25,    -44.68,
	//	//-4.5, 0.45, 0.0,  -4.02828, -39.6503, -26.65,
	//	//-4.3, 0.45, 0.0,   6.25617, -24.0357, -3.52407

	//	////34.7 l=6
	//	//-4.12, 0.3, 0, 23.7001, -41.0582, 
	//	//-4.02, 0.42, 0, 5.20047, 1.24,
	//	//-4.22, 0.44, 0, -1.26155, -35.5 7, -45.6293, 
	//	//-4.08, 0.31, 0, 10.3892, -27.7128, 18.4967

	//	//-4.55, 0.42, 0, 31.5011, 76.066,
	//	//-4.6, 0.43, 0, 19.6802, -42.8333,
	//	//-4.61, 0.37, 0, 9.72404, -32.7735, 13.1553,
	//	//-4.54, 0.42, 0, -6.06392, -15.9522, -82.4768

	//	//-3.18, 0.31, 0, 43.9736, -43.87, 
	//	//-3.31, 0.38, 0, 35.9283, -3.61596, 
	//	//-3.19, 0.45, 0, -7.97672, -19.9178, -0.105686, 
	//	//-3.36, 0.32, 0, 10.4713, 25.15, 75.61

	//};
	float nCompleted = 0.0f, normal_nCompleted = 38.5f;
	float deviation = 10.0f, deviation_ = deviation / 100.0f, sep = 1.0f;
	float r = deviation / sep, t = deviation_ / sep;
	std::vector<output> v;
	v.reserve((pow(deviation, 6) - 1) * 15);
	for (float tx = -deviation_; tx <= deviation_; tx += t)
	{
		for (float ty = -deviation_; ty <= deviation_; ty += t)
		{
			for (float tz = -deviation_; tz <= deviation_; tz += t)
			{
				for (float rx = -deviation; rx <= deviation; rx += r)
				{
					for (float ry = -deviation; ry <= deviation; ry += r)
					{
						for (float rz = -deviation; rz <= deviation; rz += r)
						{
							//cout << rz << endl;
							for (char m1 = 0; m1 < 2; m1++)
							{
								for (char m2 = 0; m2 < 2; m2++)
								{
									for (char h1 = 0; h1 < 2; h1++)
									{
										for (char h2 = 0; h2 < 2; h2++)
										{
											if (m1 + m2 + h1 + h2 == 0)continue;
											if (tx + ty + tz + rx + ry + rz == 0)continue;

											std::vector<std::vector<float>> tmp_para{ tmp1_para,tmp2_para,tmp3_para,tmp4_para };

											tmp_para.at(0).at(0) += (m1) ? tx : 0.0;
											tmp_para.at(0).at(1) += (m1) ? ty : 0.0;
											tmp_para.at(0).at(2) += (m1) ? tz : 0.0;
											tmp_para.at(0).at(3) += (m1) ? rx : 0.0;
											tmp_para.at(0).at(4) += (m1) ? ry : 0.0;

											tmp_para.at(1).at(0) += (m2) ? tx : 0.0;
											tmp_para.at(1).at(1) += (m2) ? ty : 0.0;
											tmp_para.at(1).at(2) += (m2) ? tz : 0.0;
											tmp_para.at(1).at(3) += (m2) ? rx : 0.0;
											tmp_para.at(1).at(4) += (m2) ? ry : 0.0;

											tmp_para.at(2).at(0) += (h1) ? tx : 0.0;
											tmp_para.at(2).at(1) += (h1) ? ty : 0.0;
											tmp_para.at(2).at(2) += (h1) ? tz : 0.0;
											tmp_para.at(2).at(3) += (h1) ? rx : 0.0;
											tmp_para.at(2).at(4) += (h1) ? ry : 0.0;
											tmp_para.at(2).at(5) += (h1) ? rz : 0.0;

											tmp_para.at(3).at(0) += (h2) ? tx : 0.0;
											tmp_para.at(3).at(1) += (h2) ? ty : 0.0;
											tmp_para.at(3).at(2) += (h2) ? tz : 0.0;
											tmp_para.at(3).at(3) += (h2) ? rx : 0.0;
											tmp_para.at(3).at(4) += (h2) ? ry : 0.0;
											tmp_para.at(3).at(5) += (h2) ? rz : 0.0;

											//std::vector<Livox> Sensors;
											//Sensors.reserve(nMid70 + nHorizon);
											for (size_t m = 0; m < nMid70; m++)
											{
												Mid70 tmp_mid70(tmp_para.at(m).begin());
												//Sensors.emplace_back(tmp_mid70);
											}
											for (size_t h = 0; h < nHorizon; h++)
											{
												Horizon tmp_horizon(tmp_para.at(nMid70 + h).begin());
												//Sensors.emplace_back(tmp_horizon);
											}

											switch (flag_cross_section)
											{
											case choose_arch:
												nCompleted = Arch::statistics();
												break;
											case choose_cylinder:
												nCompleted = Cylinder::statistics();
												break;
											default:
												break;
											}
											//Livox::visualiation(Sensors);
											auto percentage = (nCompleted - normal_nCompleted) / normal_nCompleted * 100.0f;
											output tmp_opt{
												.m1 = m1,
												.m2 = m2,
												.h1 = h1,
												.h2 = h2,
												.tx = tx,
												.ty = ty,
												.tz = tz,
												.rx = rx,
												.ry = ry,
												.rz = rz,
												.nCompleted = nCompleted,
												.percentage = percentage,
											};
											v.emplace_back(tmp_opt);

											cloud.clear();
											cloud_cube.clear();
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	//cout << "==================================\n"
	//	<< "Best: " << max << "(" << normal_nCompleted * (1.0f + max) << "m), Worst:" << min << "(" << normal_nCompleted * (1.0f + min) << "m)";
	int statistics_percentage[7]{ 0 };
	for (const auto& it : v)
	{
		auto p = it.percentage;
		if (p >= 0.05f)++statistics_percentage[0];
		else if (p >= 0.0f)++statistics_percentage[1];
		else if (p >= -0.05f)++statistics_percentage[2];
		else if (p >= -0.10f)++statistics_percentage[3];
		else if (p >= -0.15f)++statistics_percentage[4];
		else if (p >= -0.20f)++statistics_percentage[5];
		else ++statistics_percentage[6];
	}
	cout << "statistics\n";
	for (const auto& i : statistics_percentage)
	{
		cout << i << endl;
	}

	cout << "\noutput\n";
	std::fstream opt("sensitivity_.csv", std::ios::app | std::ios::out);
	if (!opt.good())std::cerr << "file failed!\n";
	else
	{
		for (const auto& it : v)
		{
			opt << it.m1 << ", " << it.m2 << ", " << it.h1 << ", " << it.h2 << ", "
				<< it.tx * 100.0f << ", " << it.ty * 100.0f << ", " << it.tz * 100.0f << ", "
				<< it.rx << ", " << it.ry << ", " << it.rz << ", "
				<< it.nCompleted << "m, " << it.percentage << "%,\n";
			//cout << std::fixed << setprecision(1)
			//	<< m1 << m2 << h1 << h2 << ": "
			//	<< setw(5) << tx << setw(5) << ty << setw(5) << tz << setw(6) << rx << setw(6) << ry << setw(6) << rz << setw(6) << nCompleted << "m "
			//	<< setw(6) << percentage << "%";
		}
	}
	opt.close();

	return 0;
}