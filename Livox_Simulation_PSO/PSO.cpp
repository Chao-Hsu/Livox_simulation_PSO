#include "PSO.h"

std::random_device rd;
std::mt19937_64 generator(rd());

PSO::PSO()
{
	for (size_t i = 0; i < 6; i++)p_max[i] = p_min[i] = v_max[i] = 0.0f;
	particle = 0U;
	iteration = 0U;
	limit = 0.0f;
	W = 0.0f;
	C1 = 0.0f;
	C2 = 0.0f;

	para_length = 0U;
	pos_length = 0U;
	position = nullptr;
	velocity = nullptr;
	p_best_position = nullptr;
	g_best_position = nullptr;
	tmp_fitness = nullptr;
	p_best_fitness = nullptr;
	g_best_fitness = 0.0f;
}

void PSO::set_p_max(float* p)
{
	for (unsigned i = 0; i < 6U; i++)
	{
		p_max[i] = *(p + i);
	}

	for (unsigned i = 0; i < 6U; i++)
	{
		v_max[i] = p_max[i] - p_min[i];
	}
}
void PSO::set_p_min(float* p)
{
	for (unsigned i = 0; i < 6U; i++)
	{
		p_min[i] = *(p + i);
	}

	for (unsigned i = 0; i < 6U; i++)
	{
		v_max[i] = p_max[i] - p_min[i];
	}
}
void PSO::init()
{
	particle = 50;
	iteration = 100;
	limit = 0.0f;
	W = 1.0f;
	C1 = 2.0f;
	C2 = 2.0f;

	para_length = nMid70 * 5 + nHorizon * 6;
	pos_length = para_length * particle;
	position = new float[pos_length];
	velocity = new float[pos_length];
	p_best_position = new float[pos_length];
	g_best_position = new float[para_length];
	tmp_fitness = new float[particle];
	p_best_fitness = new float[particle];
	for (unsigned mid = 0; mid < nMid70; mid++)
	{
		for (unsigned p = 0; p < particle; p++)
		{
			for (unsigned i = 0; i < 5U; i++)
			{
				auto index = mid * particle * 5U + p * 5U + i;
				velocity[index] = random(0.0f, v_max[i]);
				position[index] = random(p_min[i], p_max[i]);
			}
		}
	}
	for (unsigned hori = 0; hori < nHorizon; hori++)
	{
		for (unsigned p = 0; p < particle; p++)
		{
			for (unsigned i = 0; i < 6U; i++)
			{
				auto index = nMid70 * particle * 5U + hori * particle * 6U + p * 6U + i;
				velocity[index] = random(0.0f, v_max[i]);
				position[index] = random(p_min[i], p_max[i]);
			}
		}
	}
	compute_f();
	update();
}

void PSO::go()
{
	for (unsigned i = 0; i < iteration; i++)
	{
		move();

		compute_f();

		update();

		std::cout << std::setfill('0') << std::setw(3) << i
				  << std::setfill(' ') << std::setw(5) << g_best_fitness << "\n";
		for (size_t i = 0; i < para_length; i++)
		{
			std::cout << g_best_position[i] << ", ";
		}
		std::cout << "\n";
	}

	cloud.clear();
	for (unsigned mid70 = 0; mid70 < nMid70; mid70++)
	{
		Mid70 best00(g_best_position + 5U * mid70);
	}
	for (unsigned horizon = 0; horizon < nHorizon; horizon++)
	{
		Horizon best02(g_best_position + 5U * nMid70 + 6U * horizon);
	}

	/*std::fstream opt("diff_amount.csv", std::ios::app | std::ios::out);
	if (!opt.good())std::cerr << "file failed!\n";
	opt  << nMid70 << ", " << nHorizon << ", \n";
	opt << g_best_fitness << ", ";
	for (size_t i = 0; i < para_length; i++)
	{
		opt << g_best_position[i] << ", ";
	}
	opt << "\n";
	opt.close();*/

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
}

void PSO::move()
{
	for (unsigned mid = 0; mid < nMid70; mid++)
	{
		for (unsigned p = 0; p < particle; p++)
		{
			for (unsigned i = 0; i < 5U; i++)
			{
				auto index = mid * particle * 5U + p * 5U + i;
				auto tmp_v = W * position[index]
					+ C1 * random(0.0f, 1.0f) * (p_best_position[index] - position[index])
					+ C2 * random(0.0f, 1.0f) * (g_best_position[mid * 5U + i] - position[index]);
				velocity[index] = (abs(tmp_v) <= v_max[i]) ? tmp_v : random(0.0f, v_max[i]);

				auto tmp_p = position[index] + velocity[index];
				position[index] = (tmp_p > p_max[i] || tmp_p < p_min[i])
					? random(p_min[i], p_max[i])
					: tmp_p;
			}
		}
	}
	for (unsigned hori = 0; hori < nHorizon; hori++)
	{
		for (unsigned p = 0; p < particle; p++)
		{
			for (unsigned i = 0; i < 6U; i++)
			{
				auto index = nMid70 * particle * 5U + hori * particle * 6U + p * 6U + i;
				auto tmp_v = W * position[index]
					+ C1 * random(0.0f, 1.0f) * (p_best_position[index] - position[index])
					+ C2 * random(0.0f, 1.0f) * (g_best_position[nMid70 * 5U + hori * 6U + i] - position[index]);
				velocity[index] = (abs(tmp_v) <= v_max[i]) ? tmp_v : random(0.0f, v_max[i]);

				auto tmp_p = position[index] + velocity[index];
				position[index] = (tmp_p > p_max[i] || tmp_p < p_min[i])
					? random(p_min[i], p_max[i])
					: tmp_p;
			}
		}
	}
	//for (unsigned p = 0; p < 5; p++)
	//{
	//	for (unsigned i = 0; i < nSensor * particle; i++)
	//	{
	//		auto tmp_v = W * position[i]
	//			+ C1 * random(0.0, 1.0) * (p_best_position[i] - position[i])
	//			+ C2 * random(0.0, 1.0) * (g_best_position[i % nSensor + p * nSensor] - position[i]);
	//		velocity[i] = (abs(tmp_v) <= v_max[p]) ? tmp_v : random(0.0f, v_max[p]);
	//		auto tmp_p = position[i] + velocity[i];
	//		position[i] = (tmp_p > p_max[p] || tmp_p < p_min[p])
	//			? random(p_min[p], p_max[p])
	//			: tmp_p;
	//	}
	//}
	//for (unsigned i = nSensor * particle * 5; i < pos_length; i++)
	//{
	//	auto tmp_v = W * position[i]
	//		+ C1 * random(0.0, 1.0) * (p_best_position[i] - position[i])
	//		+ C2 * random(0.0, 1.0) * (g_best_position[i % particle] - position[i]);
	//	velocity[i] = (abs(tmp_v) <= v_max[5]) ? tmp_v : random(0.0f, v_max[5]);
	//	auto tmp_p = position[i] + velocity[i];
	//	position[i] = (tmp_p > p_max[5] || tmp_p < p_min[5])
	//		? random(p_min[5], p_max[5])
	//		: tmp_p;
	//}
}

void PSO::compute_f()
{
	for (unsigned i = 0; i < particle; i++)
	{
		cloud.clear();
		for (unsigned mid = 0; mid < nMid70; mid++)
		{
			Mid70 tmp(position + i * 5U + mid * particle * 5U);
		}
		for (unsigned hori = 0; hori < nHorizon; hori++)
		{
			Horizon tmp(position + nMid70 * particle * 5U + i * 6U + hori * particle * 6U);
		}
		switch (flag_cross_section)
		{
		case choose_arch:
			tmp_fitness[i] = Arch::statistics();
			break;
		case choose_cylinder:
			tmp_fitness[i] = Cylinder::statistics();
			break;
		default:
			break;
		}
	}
}

void PSO::update()
{
	for (unsigned i = 0; i < particle; i++)
	{
		if (tmp_fitness[i] > p_best_fitness[i])
		{
			p_best_fitness[i] = tmp_fitness[i];
			for (unsigned mid = 0; mid < nMid70; mid++)
			{
				for (unsigned para = 0; para < 5U; para++)
				{
					auto index = mid * particle * 5U + i * 5U + para;
					p_best_position[index] = position[index];
				}
			}
			for (unsigned hori = 0; hori < nHorizon; hori++)
			{
				for (unsigned para = 0; para < 6U; para++)
				{
					auto index = nMid70 * particle * 5U + hori * particle * 6U + i * 6U + para;
					p_best_position[index] = position[index];
				}

			}
		}
	}
	auto max_ptr = std::max_element(tmp_fitness, tmp_fitness + particle);
	auto max_index = max_ptr - tmp_fitness;
	if (*max_ptr > g_best_fitness)
	{
		g_best_fitness = tmp_fitness[max_index];
		for (unsigned mid = 0; mid < nMid70; mid++)
		{
			for (unsigned para = 0; para < 5U; para++)
			{
				auto index = mid * particle * 5U + max_index * 5U + para;
				g_best_position[mid * 5U + para] = position[index];
			}
		}
		for (unsigned hori = 0; hori < nHorizon; hori++)
		{
			for (unsigned para = 0; para < 6U; para++)
			{
				auto index = nMid70 * particle * 5U + hori * particle * 6U + max_index * 6U + para;
				g_best_position[nMid70 * 5U + hori * 6U + para] = position[index];
			}

		}
	}
}

float PSO::random(float lower, float upper)
{
	std::uniform_real_distribution<double> unif(lower, upper);
	return floor(unif(generator) * 100.0) / 100.0;
}