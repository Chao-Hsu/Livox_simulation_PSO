#include "PSO.h"

std::random_device rd;
std::mt19937_64 generator(rd());

PSO::PSO()
{
	for (size_t i = 0; i < 6; i++)p_max[i] = p_min[i] = v_max[i] = 0.0f;

	particle = 50;
	iteration = 1000.0f;
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
	g_best_fitness = 0.0f;
	for (unsigned i = 0; i < pos_length; i++)position[i] = velocity[i] = 0.0f;
	for (unsigned i = 0; i < para_length; i++)p_best_position[i] = g_best_position[i] = 0.0f;
	for (unsigned i = 0; i < particle; i++)tmp_fitness[i] = 0.0f;
}


void PSO::go()
{
	move();
}

void PSO::move()
{
	for (unsigned m = 0; m < nMid70; m++)
	{
		for (unsigned p = 0; p < particle; p++)
		{
			for (unsigned i = 0; i < 5U; i++)
			{
				auto index = m * particle * 5U + p * 5U + i;
				auto tmp_v = W * position[index]
					+ C1 * random(0.0, 1.0) * (p_best_position[index] - position[index])
					+ C2 * random(0.0, 1.0) * (g_best_position[m * 5U + i] - position[index]);
				velocity[index] = (abs(tmp_v) <= v_max[i]) ? tmp_v : random(0.0f, v_max[i]);

				auto tmp_p = position[index] + velocity[index];
				position[index] = (tmp_p > p_max[i] || tmp_p < p_min[i])
					? random(p_min[i], p_max[i])
					: tmp_p;
			}
		}
	}
	for (unsigned h = 0; h < nHorizon; h++)
	{
		for (unsigned p = 0; p < particle; p++)
		{
			for (unsigned i = 0; i < 6U; i++)
			{
				auto index = nMid70 * particle * 5U + h * particle * 6U + p * 6U + i;
				auto tmp_v = W * position[index]
					+ C1 * random(0.0, 1.0) * (p_best_position[index] - position[index])
					+ C2 * random(0.0, 1.0) * (g_best_position[nMid70 * 5U +  h * 6U + i] - position[index]);
				velocity[index] = (abs(tmp_v) <= v_max[p]) ? tmp_v : random(0.0f, v_max[p]);

				auto tmp_p = position[index] + velocity[index];
				position[index] = (tmp_p > p_max[p] || tmp_p < p_min[p])
					? random(p_min[p], p_max[p])
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
	for (unsigned i = 0; i < nSensor * particle; i++)
	{

	}
}

float PSO::random(float lower, float upper)
{
	std::uniform_real_distribution<double> unif(lower, upper);
	return floor(unif(generator) * 100.0) / 100.0;
}