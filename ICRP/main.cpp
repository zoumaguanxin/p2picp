#include<fstream>
#include<iostream>
#include<sstream>
#include<string>
#include <vector>
#include <ctime>
//*********************************
#include <Eigen/Dense>
#include <Eigen/Geometry>
//*********************************

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
using namespace std;
//using Eigen::MatrixXd;
template <class T>
int getArrayLen(T& a){
	return (sizeof(a)/sizeof(T));
}
//定义位姿类

class pose{
	public:
		pose(){x=0;
			y=0;
			z=0;
			thetax=0;
			thetay=0;
			thetaz=0;}
		pose(double xx,double yy,double zz,double thetaxx,double thetayy,double thetazz)
		{
			x=xx;
			y=yy;
			z=zz;
			thetax=thetaxx;
			thetay=thetayy;
			thetaz=thetazz;
		}
		void getpose(){
			cout<<" "<<x<<" "<<y<<" "<<z<<endl;
			cout<<" "<<thetax<<" "<<thetay<<" "<<thetaz<<endl;

		}
		double x,y,z;
		double thetax,thetay,thetaz;

	};
//定义点云类
class PointXYZ0{
public:
	double x,y,z,h;
		PointXYZ0(){
			x=0;y=0;z=0;h=0;}
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main(int arg, char** argv)
{

	//*********************************************************************************
	//定义基本的变量
	//*********************************************************************************
    char dir[100]="/home/shaoan/projects/SLAM6D/dat_et4/";//不能写成string dir="H:\\"；在下面sptintf_s输出时会出现乱码
	const int nfilecounter=2;//文件数目
	const float r_filter=700;
	char poseFileName[255];
	char pointFileName[255];
	int nPoint=32000;
	int n;//可能存放点云数
	srand((unsigned int) (time (NULL)));//播种随机种子
	PointXYZ0 point[1];//在windows下，栈是向低地址扩展的数据结构，是一块连续的内存区域，栈顶的地址和栈的最大容量是系统预先规定好的，能从栈获得的空间较小。
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);//申请了一个pcl::PointCloud<pcl::PointXYZ>类的指针
	pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pose pose_odometry[nfilecounter];
	//存放上一次求出位姿
	Eigen::Matrix3f R_last;
	Eigen::Vector3f t_last;
	//存放预测的位姿
	Eigen::Matrix3f R_predict;
	Eigen::Vector3f t_predict;
	//存放里程计测得的两次之间的变化
	Eigen::Matrix3f R_delta;
	Eigen::Vector3f t_delta;
	//存放上一次里程计的数据
	Eigen::Matrix3f R_odometry_last;
	Eigen::Vector3f t_odometry_last;
	//存放当前里程计数据
	Eigen::Matrix3f R_odometry;
	Eigen::Vector3f t_odometry;
	//存放当前滤波后点云数目和上一次滤波后点云数目
	int nPoint_current;
	int nPoint_last;

	//cout<<point[4].x<<" "<<point[4].y<<" "<<point[4].z;
	//读取姿态和3D点
	//****************************************************
	int filecounter=0;
       pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr2 (new pcl::PointCloud<pcl::PointXYZ>);//如果需要评估请取消注释，并注释掉128行、441行
	for(int nfile=0;nfile<nfilecounter;nfile++)
	{
		if(nfile>0) filecounter++;
	    sprintf(poseFileName,"%sscan%.3d.pose",dir,nfile);//sprintf是把后面格式的文字输出到缓存，这里poseFileName是缓存的地址,.3d表示了
	    sprintf(pointFileName,"%sscan%.3d.3d",dir,nfile);

	    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	   // pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr2 (new pcl::PointCloud<pcl::PointXYZ>);

	//***************************************************
	//读取pose
	//***************************************************

	    cout<<"reading pose from "<<poseFileName<<endl;
	    ifstream is(poseFileName,ios_base::in);
	    double rpos[3],rposTheta[3];//用来存放pose
	    if(is.good()){
	       for(unsigned int i=0;i<3;is>>rpos[i++]);
	       for(unsigned int i=0;i<3;is>>rposTheta[i++]);
	    }
	    else{cout<<"ERROR: Cannot open file 'scan000.pose'."<<endl;}
	    is.close();
	//显示读取的结果
	    for(unsigned int i=0;i<3;cout<<rpos[i++]<<" ");cout<<endl;
	    for(unsigned int i=0;i<3;cout<<rposTheta[i++]<<" ");cout<<endl;

	//*************************************************
	//欧拉角转换为旋转矩阵,注意Eigen使用的角度均为弧度
	//*************************************************

	   t_odometry<<rpos[0],rpos[1],rpos[2];
	   R_odometry=Eigen::AngleAxisf(pcl::deg2rad(rposTheta[2]), Eigen::Vector3f::UnitZ())
              * Eigen::AngleAxisf(pcl::deg2rad(rposTheta[1]), Eigen::Vector3f::UnitY())
	          * Eigen::AngleAxisf(pcl::deg2rad(rposTheta[0]), Eigen::Vector3f::UnitX());
	    cout<<"current Rotation Matix is equal to:"<<endl
		<<R_odometry<<endl
	    <<"the tranlate data from odometry"<<endl
		<<t_odometry<<endl;

		//打开点云文件读取点云数据
		ifstream file(pointFileName,ios_base::in);
		if (file.good()){

		//******************
		//读取文件的初始信息
		//******************

		double size1;file>>size1;cout<<size1;
		char xxx;
		file>>xxx;
		cout<<xxx;double size2;
		file>>size2;cout<<size2<<endl;
		n=size1*size2;//


		//**********************************
		//读取点云
		//**********************************

		for(int i=0;i<nPoint;i++)
		{
				if(file.good())
				{
					pcl::PointXYZ cloud;
					file>>cloud.x;file>>cloud.y;file>>cloud.z;file>>point[0].h;
					//cout<<cloud.x<<" "<<cloud.y<<" "<<cloud.z<<" "<<i<<endl;
					float distance=sqrt(pow(cloud.x,2)+pow(cloud.y,2)+pow(cloud.z,2));
					if(distance<=r_filter)
					{
						if(filecounter==0)
						global_cloud_ptr->points.push_back(cloud);//把点云堆进去
						else
						basic_cloud_ptr2->points.push_back(cloud);
					}
				}
				else{cout<<"file were read completely"<<endl;break;
				}
		}
		}
		else{cout<<"ERROR: Cannot open file"<<pointFileName<<endl;}


		//局部地图建立
		if(nfile==0)
		{
			nPoint_last=global_cloud_ptr->points.size();
			cout<<nPoint_last<<endl;
		}
		else
		{
			nPoint_current=basic_cloud_ptr2->points.size();
			cout<<nPoint_current<<endl;
		}

		if((filecounter<=2)&&(filecounter>=1))
		{
			local_cloud_ptr=global_cloud_ptr;
		}
		else if(filecounter>=3)
		{
			for(int i=0;i<(nPoint_current+nPoint_last);i++)
			{
				local_cloud_ptr->points.push_back(global_cloud_ptr->points[global_cloud_ptr->points.size()-(nPoint_current+nPoint_last)+i]);
			}
		}
		nPoint_last=nPoint_current;

	//位姿预测,效果不好，目前没有看到起积极作用
	/********************************************
	if(nfile==0)
	{
		R_last=R_odometry.transpose();
		t_last=t_odometry;
	}
	else{
		R_delta=R_odometry*R_odometry_last.transpose();
		t_delta=t_odometry-t_odometry_last;
		//R_predict=R_delta;
		//t_predict=t_delta;
		R_predict=R_delta*R_last.transpose();
		t_predict=t_last+t_delta;
	}
	**********************************************************************/



	//***************************************************
	//kdtree搜索+ICP(vanilla-picky)
	//******************************************************

//ICP算法
       //***************************
       //算法时间计时起点
       //***************************
        time_t begin,end,begin1,end1;
        begin=clock();
	double time_sum_search=0;

/****************************************************************************************************/
//数据点关联，方案1

	if(filecounter>=1){
	const int k=1000;//设置搜搜索点云的个数
	const float search_r_low=0;
	float search_r_up=1010;
	float d_threshold=10;
	Eigen::Vector3f t_update=t_odometry;
	Eigen::Matrix3f R_update=R_odometry.transpose();
        //Eigen::Vector3f t_update=t_predict;
	//Eigen::Matrix3f R_update=R_predict.transpose();
	pcl::PointXYZ initPoints[k];//随机找出k个点云并存放在这个类数组中
	pcl::PointXYZ points_transformed;//并应用初始旋转
	pcl::PointXYZ matchedPoints[k];//匹配得到的点
	pcl::PointXYZ matchedPoints1[k];
	size_t K=1;
	vector<int> pointIdxNKNSearch(K);//用来存放搜索到的点的index
	vector<int> pointIdxNKNSearch1(K);//用来存放搜索到的点的index
	vector<float> pointNKNSquaredDistance(K);//存放搜索到的点到当前的欧式距离
	vector<float> pointNKNSquaredDistance1(K);//存放搜索到的点到当前的欧式距离
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1;
	kdtree.setInputCloud(local_cloud_ptr);//把点云按照kdtree形式存储
	float error=200000;
	float error2=0;
	int convergence_counter=0;
	float sigma=k*2.1;//收敛时允许的误差
	int iterative_counter=0;
	int counter_max=60;
	while(convergence_counter<=2&&iterative_counter<counter_max){
	  begin1=clock();
	    Eigen::Vector3f v1;
	    Eigen::Vector3f v2;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr middle_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		for(int i=0;i<basic_cloud_ptr2->points.size();i++)
		{
			v1(0)=basic_cloud_ptr2->points[i].x;
			v1(1)=basic_cloud_ptr2->points[i].y;
			v1(2)=basic_cloud_ptr2->points[i].z;
			v2=R_update*v1;//应用初始的旋转
			points_transformed.x=v2(0)+t_update(0);
			points_transformed.y=v2(1)+t_update(1);
			points_transformed.z=v2(2)+t_update(2);
			middle_cloud_ptr->points.push_back(points_transformed);
		}
		kdtree1.setInputCloud(middle_cloud_ptr);
	bool flag=1;
	int *p=new int;
	int i=0;
	int counter=0;
	while(i<k)
	{
		int j;
		unsigned int R;
		unsigned int t;
		 R = RAND_MAX-(RAND_MAX+1)%nPoint_current; //去除尾数
                 t = rand();
                while( t > R ) t = rand();
L1:            j = t % nPoint_current;
         //     j=(int)nPoint_current*rand()/(RAND_MAX+1);//这样生成随机数程序会无法运行
		if (counter==0) p[counter]=j;
		else{
			for(int m=0;m<counter;m++)
		      {
			if(p[m]==j)
			    {
				goto L1;
			    }
		      }
		    p[++counter]=j;
		   }
		kdtree.nearestKSearch (middle_cloud_ptr->points[j], K, pointIdxNKNSearch, pointNKNSquaredDistance);
		//ICRP
		kdtree1.nearestKSearch(local_cloud_ptr->points[pointIdxNKNSearch[0]],K,pointIdxNKNSearch1, pointNKNSquaredDistance1);
		double d;
		d=pow(abs(middle_cloud_ptr->points[pointIdxNKNSearch1[0]].x-middle_cloud_ptr->points[j].x),2)
			+pow(abs(middle_cloud_ptr->points[pointIdxNKNSearch1[0]].y-middle_cloud_ptr->points[j].y),2)
			+pow(abs(middle_cloud_ptr->points[pointIdxNKNSearch1[0]].z-middle_cloud_ptr->points[j].z),2);
		d=sqrt(d);
		if(d<d_threshold){
			initPoints[i]=basic_cloud_ptr2->points[j];
			matchedPoints[i]=local_cloud_ptr->points[pointIdxNKNSearch[0]];
			//用于显示所搜到的最近点的距离
			/*
				     for (size_t l = 0; l < pointIdxNKNSearch.size (); ++l)
					 {

                        std::cout << "    "  <<local_cloud_ptr->points[ pointIdxNKNSearch[l] ].x
                        << " " <<  local_cloud_ptr->points[ pointIdxNKNSearch[l] ].y
                        << " " << local_cloud_ptr->points[ pointIdxNKNSearch[l] ].z
                        << " (squared distance: " << pointNKNSquaredDistance[l] << ")" << endl;
					 }
			*/
			i++;
			}

		}
		end1=clock();
		time_sum_search=double(end1-begin1)/CLOCKS_PER_SEC;


	Eigen::Vector3f centriod_initPoints(0,0,0);//初始点云中心,当向量维数小于4的时候，这样的初始化也是被允许的
	Eigen::Vector3f centriod_matchedPoints(0,0,0);//匹配到的点云中心
	Eigen::Matrix3f U,V,S,R_svd;
	Eigen::Vector3f t_translate;


	Eigen::MatrixXf W;
	for(int i=0;i<k;i++)
	{   Eigen::Vector3f temV;
	    temV<<initPoints[i].x,initPoints[i].y,initPoints[i].z;//p
		centriod_initPoints+=temV;
		temV<<matchedPoints[i].x,matchedPoints[i].y,matchedPoints[i].z;//y
		centriod_matchedPoints+=temV;
	}
	centriod_initPoints/=(float)k;
	centriod_matchedPoints/=(float)k;
	for(int i=0;i<k;i++)
	{
		 Eigen::Vector3f temV1;
		 Eigen::Vector3f temV2;
		 temV1<<initPoints[i].x,initPoints[i].y,initPoints[i].z;
		 temV1-=centriod_initPoints;
		 temV2<<matchedPoints[i].x,matchedPoints[i].y,matchedPoints[i].z;
		 temV2-=centriod_matchedPoints;
		 if (i==0) {W=temV2*temV1.transpose();}
		 else {W+=temV2*temV1.transpose();}	//y*p'
	}
	W/=(float)k;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(W,Eigen::ComputeFullU|Eigen::ComputeFullV);
	U=svd.matrixU();//不要写建议写成Eigen::Matrix3f U=svd.matrixU();
	V=svd.matrixV();
	S<<1,0,0,
	   0,1,0,
	   0,0,U.determinant()*V.determinant();
	R_svd=U*S*V.transpose();
	t_translate=centriod_matchedPoints-R_svd*centriod_initPoints;//y-R*p.求出的是initPoints点所在坐标系到matchedPoints点所在坐标系位移,注意这个位移是表示在matchedPoints坐标系下的,这里也就是global坐标系下
	cout<<"find the rotation Matrix using SVD method:"<<endl
		<<R_svd<<endl
		<<"平移："<<endl
		<<t_translate<<endl;
	R_update=R_svd;
	t_update=t_translate;
	error=0;
	for(int j=0;j<k;++j)
	{
		Eigen::Vector3f temV1;
		Eigen::Vector3f temV2;
		temV1<<initPoints[j].x,initPoints[j].y,initPoints[j].z;
		temV2<<matchedPoints[j].x,matchedPoints[j].y,matchedPoints[j].z;
        temV1=R_update*temV1+t_update-temV2;
		error+=sqrt(temV1.dot(temV1));
	}
	if(abs(error-error2)<100&&(error<sigma)) convergence_counter+=1;
	//else if(((error-error2)<0)&&(convergence_counter>=1)) convergence_counter-=1;
	error2=error;
	if(search_r_up>=30) search_r_up-=25;//vanilla ICP的核心思想，半径逐渐减小。
	//else if(search_r_up>=3) search_r_up-=0.5;
	//if(d_threshold>2) d_threshold-=2;
	cout<<"the cureent error is equal to:"<<error<<endl;
	iterative_counter++;
	cout<<"the number of iterative:"<<iterative_counter<<endl;
	}//ICP算法的结尾。while(convergence_counter<=2)
	//保存当前数据供下一次使用
	R_last=R_update;
	t_last=t_update;
	/*************************************************************************************************/


	//******************************
	//将观测到的点统一到全局坐标系下
	//******************************
	for(int j=0;j<basic_cloud_ptr2->points.size();++j)
	{
		Eigen::Vector3f temV;
		temV<<basic_cloud_ptr2->points[j].x,basic_cloud_ptr2->points[j].y,basic_cloud_ptr2->points[j].z;
		temV=R_update*temV;
		basic_cloud_ptr2->points[j].x=temV(0)+t_update(0);
		basic_cloud_ptr2->points[j].y=temV(1)+t_update(1);
		basic_cloud_ptr2->points[j].z=temV(2)+t_update(2);
		//global_cloud_ptr->points.push_back(basic_cloud_ptr2->points[j]);
		}
	}//if(filecounter>=1)的结尾

	//保存当前数据，供下一次使用
	R_odometry_last=R_odometry;
	t_odometry_last=t_odometry;


	//*****************************
       //程序运行时间评估
       //*****************************

         end=clock();
	 cout<<"search runtime"<<double(end1-begin)/CLOCKS_PER_SEC<<endl;
         cout<<"runtime: "<<double(end-begin)/CLOCKS_PER_SEC<<endl;

}//for(filecounter=0;filecounter<nfilecounter;filecounter++)的结尾



	//************************************************
	//评估当前求解的好坏
	//************************************************
	const int n_iner=5000;
	size_t K=1;
	float error_total=0.0;
	vector<int> pointIdxNKNSearch(K);//用来存放搜索到的点的index
	vector<float> pointNKNSquaredDistance(K);//存放搜索到的点到当前的欧式距离
	vector<float> pointsDistance(nPoint_current);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(global_cloud_ptr);
	for(size_t i=0;i<nPoint_current;++i){
	  kdtree.nearestKSearch (basic_cloud_ptr2->points[i], K,pointIdxNKNSearch, pointNKNSquaredDistance);
	  pointsDistance[i]=pointNKNSquaredDistance[0];
	}

	sort(pointsDistance.begin(),pointsDistance.end());
	for(size_t i=0;i<n_iner;i++){
	 // cout<<pointsDistance[i]<<endl;
	  error_total+=pointsDistance[i];
	}
	cout<<"the total error of inner points is:"<<error_total<<endl;




//***************************************************************
	for(int j=0;j<basic_cloud_ptr2->points.size();++j)
	{
		global_cloud_ptr->points.push_back(basic_cloud_ptr2->points[j]);

	}
//*********************************************************/





	//*************************************************
	//可视化
	//*************************************************
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr vc (new pcl::PointCloud<pcl::PointXYZ>);//其中new pcl::PointCloud<pcl::PointXYZ>必不可少。并检查这段程序是否可以优化
	viewer = simpleVis(global_cloud_ptr);
	while (!viewer->wasStopped ())
	  {
		  if(!viewer->wasStopped ())          viewer->spinOnce (100);
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }

	 return 0;
}


