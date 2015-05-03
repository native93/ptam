#include <VideoSource.h>
//#include<ardrone_autonomy/Navdata.h>
using namespace message_filters;



VideoSource::VideoSource(ros::NodeHandle &nh_): nh(nh_),it_(nh), is_frame_available(false)
{
//VideoSource::VideoSource(ros::NodeHandle &nh_) : nh(nh_), it_(nh), is_frame_available(false)

    ros::NodeHandle nh_private("~");
    //read the input topic name from a parameter set by the launch file
    std::string input_topic_name;
//message_filters::Subscriber<sensor_msgs::Image>sub(it_,input_topic_name, 1);
  nh_private.getParam("input_topic_name", input_topic_name);
    sub = it_.subscribe(input_topic_name, 10, &VideoSource::imageCallback, this);

//  sub.subscribe(it_,input_topic_name, 110);
//  altitude_sub.subscribe(nh, "/ardrone/navdata_altitude", 100);
//altitude_sub.subscribe(nh,"/ardrone/navdata_altitude",100);
//nav_sub(nh,"/ardrone/navdata",10); 
//TimeSynchronizer<sensor_msgs::Image,ardrone_autonomy::navdata_altitude> sync(sub,altitude_sub, 10);
//message_filters::Synchronizer<MySyncPolicy>sync(MySyncPolicy(20),sub,altitude_sub);
//sync.connectInput(sub,altitude_sub);

//sync.registerCallback(boost::bind(&VideoSource::callback,this, _1, _2));


//altitude_sub=nh.subscribe("/ardrone/navdata_altitude",100,&VideoSource::altitude_callback,this);
    //Read the frame width and height again using parameters.
// sync.registerCallback(&VideoSource::callback,this);
    int frame_width;
    int frame_height;
    nh_private.getParam("frame_width", frame_width);
    nh_private.getParam("frame_height", frame_height);
    mirSize = CVD::ImageRef(frame_width, frame_height);
    //odom_sub=nh.subscribe("/odom",10,&VideoSource::odom_callback,this);
    //wait for an image to come
    ROS_INFO("Waiting for the first image");
    //ros::spin();
    ros::Rate loop_rate(15);
    while(!is_frame_available)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ic=0;oc=0;
}

void VideoSource::callback( const sensor_msgs::ImageConstPtr &img_msg,const ardrone_autonomy::navdata_altitude::ConstPtr &msg)
{

//fprintf(stderr,"inside\n");
alti_raw=*msg;
   img = *img_msg;
   is_frame_available = true;
 
}
CVD::ImageRef VideoSource::Size()
{ 
    return mirSize;
};
/*
void VideoSource::altitude_callback(const ardrone_autonomy::navdata_altitude::ConstPtr &msg)
{
alti_raw=*msg;
//fprintf(stderr,"altitude:%d\n",altitude);
}
void VideoSource::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_pose.position.x=msg->pose.pose.position.x;
    odom_pose.position.y=msg->pose.pose.position.y;
    odom_pose.position.z=msg->pose.pose.position.z;
    odom_pose.orientation.x=msg->pose.pose.orientation.x;
    odom_pose.orientation.y=msg->pose.pose.orientation.y;
    odom_pose.orientation.z=msg->pose.pose.orientation.z;
    odom_stamp=msg->header.stamp;
    //printf("o=%lf\n", image_stamp.toSec());
    oc++;
}


void VideoSource::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    image_stamp=img_msg->header.stamp;
    //std::cout<"i="<<image_stamp.toSec()<<std::endl;
    ////std::cout<"i="<<std::endl;
   // printf("i=%lf\n", image_stamp.toSec());
    img = *img_msg;
    is_frame_available = true;
    ic++;
ros::spinOnce();
}
*/
void VideoSource::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
  //  image_stamp=img_msg->header.stamp;
    //std::cout<"i="<<image_stamp.toSec()<<std::endl;
    ////std::cout<"i="<<std::endl;
   // printf("i=%lf\n", image_stamp.toSec());
    img = *img_msg;
    is_frame_available = true;
 //   ic++;
//ros::spinOnce();
}

bool VideoSource::GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB,float* pos_x,float* pos_y,bool* first_kf_ptr,bool* second_kf_ptr)
{
    is_frame_available = false;
//message_filters::Synchronizer<MySyncPolicy>sync(MySyncPolicy(20),sub,altitude_sub);

//sync.registerCallback(boost::bind(&VideoSource::callback,this, _1, _2));

    //wait for an image to come
    //    ROS_INFO("Waiting for the next image");
    ros::Rate loop_rate(60); //twice as fast
    ros::Time n = ros::Time::now();
    while(!is_frame_available)
    {
	ros::spinOnce();

//	ros::spinOnce();
//	loop_rate.sleep();
//	if((ros::Time::now()-n).toSec()>2)
//	{
//	    printf("returning\n");
//	    return false;
//	}
    }

    //now that we have the image we should return it.
    imBW.resize(mirSize);
    imRGB.resize(mirSize);

    //    ROS_INFO("img.data.size() is %d", img.data.size());
    CVD::Rgb<CVD::byte>* img_data_rgb = new CVD::Rgb<CVD::byte>[img.data.size()/3];
    CVD::byte* img_data_grey = new CVD::byte[img.data.size()/3];
    for(int i = 0; i < (int)(img.data.size()/3); i++)
    {
	img_data_rgb[i].red = img.data[3*i];
	img_data_rgb[i].blue = img.data[3*i+1];
	img_data_rgb[i].green = img.data[3*i+2];
	img_data_grey[i] = (img.data[3*i] + img.data[3*i+1] + img.data[3*i+2])/3;
    }

    CVD::BasicImage<CVD::Rgb<CVD::byte> > img_temp_rgb(img_data_rgb, mirSize);
    imRGB.copy_from(img_temp_rgb);
    CVD::BasicImage<CVD::byte> img_temp_grey(img_data_grey, mirSize);
    imBW.copy_from(img_temp_grey);
time_sec=image_stamp.toSec();
//fprintf(stderr,"time%lf\n",time_sec);

//fprintf(stderr,"frame:%lf,ultrasound:%lf,dif:%lf\n",img.header.stamp.toSec(),alti_raw.header.stamp.toSec(),img.header.stamp.toSec()-alti_raw.header.stamp.toSec());
//*altitude=alti_raw.altitude_raw;`

    return true;
}
