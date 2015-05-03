#include <VideoSource.h>

using namespace message_filters;



VideoSource::VideoSource(ros::NodeHandle &nh_): nh(nh_),it_(nh), is_frame_available(false)
{

    ros::NodeHandle nh_private("~");
    //read the input topic name from a parameter set by the launch file
    std::string input_topic_name;
    //message_filters::Subscriber<sensor_msgs::Image>sub(it_,input_topic_name, 1);
    nh_private.getParam("input_topic_name", input_topic_name);
    sub = it_.subscribe(input_topic_name, 10, &VideoSource::imageCallback, this);

    int frame_width;
    int frame_height;
    nh_private.getParam("frame_width", frame_width);
    nh_private.getParam("frame_height", frame_height);
    mirSize = CVD::ImageRef(frame_width, frame_height);

    ROS_INFO("Waiting for the first image");

    ros::Rate loop_rate(15);
    while(!is_frame_available){
        ros::spinOnce();
        loop_rate.sleep();
    }

    ic=0;oc=0;
}

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

CVD::ImageRef VideoSource::Size()
{ 
    return mirSize;
};

bool VideoSource::GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB,float* pos_x,float* pos_y,bool* first_kf_ptr,bool* second_kf_ptr)
{
    is_frame_available = false;
    ros::Rate loop_rate(60); //twice as fast
    ros::Time n = ros::Time::now();
    while(!is_frame_available)
    	ros::spinOnce();

    //now that we have the image we should return it.
    imBW.resize(mirSize);
    imRGB.resize(mirSize*3);

    //    ROS_INFO("img.data.size() is %d", img.data.size());
    CVD::Rgb<CVD::byte>* img_data_rgb = new CVD::Rgb<CVD::byte>[img.data.size()];
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
    return true;
}
