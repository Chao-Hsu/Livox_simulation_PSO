#include "Livox.h"

//char flag_cross_section = choose_cylinder;
//pcl::PointCloud<pcl::PointXYZ> cloud;


int main(int argc, char* argv[])
{
	flag_cross_section = choose_arch;

	Mid70 tmp(-4.09, 0.4782, 0.0, 6.26607, -10.3483);
	Mid70 tmp3(-4.23, 0.27, 0.0, 30.5737, -80.2208);
	Horizon tmp2(-4.17, 0.49, 0.0, -8.25457, -36.11, -8.2148);
	Horizon tmp4(-4.21, 0.41, 0.0, 7.78494, -34.0056, -2.77178);
	Livox* L[4] = { &tmp,&tmp2,&tmp3,&tmp4 };
	//tmp4(1, 2, 3, 4, 5, 6);

	for (size_t i = 0; i < 1; i++)
	{
		//L[i]->print();
	}
	std::cout << nMid70 << " " << nHorizon;

	pcl::visualization::PCLVisualizer viewer("cloud");
	viewer.setBackgroundColor(0, 0, 0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_ptr = cloud.makeShared();

	pcl::PassThrough<pcl::PointXYZ> passTrans;
	passTrans.setInputCloud(cloud_ptr);
	passTrans.setFilterFieldName("z");
	passTrans.setFilterLimits(0.0f, 50.0f);
	passTrans.filter(*cloud_ptr);

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_ptr, "z");
	viewer.addPointCloud<pcl::PointXYZ>(cloud_ptr, fildColor, "sample");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample");

	while (!viewer.wasStopped()) {
		viewer.spinOnce(1000);
	}

	system("pause");

	return 0;
}