#ifndef MYPOINTTYPE_H
#define MYPOINTTYPE_H
 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
 
struct MyPointType
{
	PCL_ADD_POINT4D;	// This adds the members x,y,z which can also be accessed using the point (which is float[4])

	float range;
	float velocity;
	int16_t intensity;
	//float rang_max;
	//float range_res;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;	// 确保new操作符对齐操作
}EIGEN_ALIGN16;		// 16位SSE对齐

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,// 注册点类型宏
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, range, range)	
	(float, velocity, velocity)
	(int16_t, intensity, intensity)
	//(float, range_max, range_max)
	//(float, range_res, range_res)
	)
 
#endif
