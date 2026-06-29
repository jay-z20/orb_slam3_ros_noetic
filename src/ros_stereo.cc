/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
*
*/

#include "custom_common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(const string& working_path): img_count_{0}, initialised_{false}, working_path_{working_path}, tf_listener_{tf_buffer_} {};

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);

    void TimerCallback(const ros::WallTimerEvent& event);

private:
    int img_count_;
    bool initialised_;
    string working_path_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    } 

    string working_path;
    ROS_INFO_STREAM(node_name);
    ROS_INFO_STREAM(node_name << "/working_path");
    node_handler.param<std::string>(node_name + "/working_path", working_path, ros::package::getPath("orb_slam3_ros") + "/output/");
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/base_link_frame_id", base_link_frame_id, "base_link");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    if (working_path[-1] != '/')
        working_path += "/";
    ROS_INFO_STREAM(working_path);
    keyframes_poses_file = working_path + "KeyframesPoses.txt";
    loop_closure_edges_file = working_path + "LoopClosureEdges.txt";

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::STEREO;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb(working_path);

    initial_pose_client = node_handler.serviceClient<orb_slam3_ros::GetTransform>("get_first_pose");
    message_filters::Subscriber<sensor_msgs::Image> sub_img_left(node_handler, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_right(node_handler, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    timer = node_handler.createWallTimer(ros::WallDuration(15.0), &ImageGrabber::TimerCallback, &igb);

    ros::spin();

    // Stop all threads
    cout << "main working_path:" << working_path << endl;
    pSLAM->Shutdown(working_path);
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight)
{
    timer.stop();

    if (!initialised_)
    {
        robot2camera.setIdentity();
        try
        {
            geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(base_link_frame_id, cam_frame_id, ros::Time(0));
            tf2::fromMsg(tf_msg.transform, robot2camera);
            initialised_ = true;
        }
        catch (const tf2::LookupException& e)
        {
            ROS_WARN_STREAM("[ORB-SLAM3-ROS->Initialiser:] TF listener exception: " << e.what());
            ROS_WARN_STREAM("[ORB-SLAM3-ROS->Initialiser:] The transform between the robot and the camera has not been found, and therefore robot2camera is equal to the identity.");
            initialised_ = true;
        }
        catch (const tf2::TransformException& e)
        {
            ROS_WARN_STREAM("[ORB-SLAM3-ROS->Initialiser:] TF listener exception: " << e.what());
            initialised_ = false;
        }
        catch (...)
        {
            ROS_WARN("Unknowns exceptions");
            initialised_ = false;
        }

        orb_slam3_ros::GetTransform srv;
        if (initial_pose_client.call(srv))
        {
            tf2::fromMsg(srv.response.tf.transform, world2initial);
            ROS_WARN_STREAM("[ORB-SLAM3-ROS->Initialiser:] GetTransform service call successful. The initial pose is as follows:" << std::endl <<
                            "tx: " << world2initial.getOrigin().getX() << std::endl << 
                            "ty: " << world2initial.getOrigin().getY() << std::endl << 
                            "tz: " << world2initial.getOrigin().getZ() << std::endl << 
                            "qx: " << world2initial.getRotation().getX() << std::endl << 
                            "qy: " << world2initial.getRotation().getY() << std::endl << 
                            "qz: " << world2initial.getRotation().getZ() << std::endl << 
                            "qw: " << world2initial.getRotation().getW());
        }
        else
        {
            ROS_WARN_STREAM("[ORB-SLAM3-ROS->Initialiser:] Failed to call GetTransform service. The initial odometry will be assumed as the identity.");
            world2initial.setIdentity();
        }
    }

    double t_resize = 0;
    double t_rect = 0;
    double t_track = 0;

    ros::Time msg_time = msgLeft->header.stamp;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    #ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #endif

    // ORB-SLAM3 runs in TrackStereo()
    Sophus::SE3f Tcw = pSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, msg_time.toSec());

    #ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        t_track = t_resize + t_rect + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        pSLAM->InsertTrackTime(t_track);
    #endif

    publish_topics(msg_time);

    pSLAM->SaveKeyFrameTrajectoryCustom(keyframes_poses_file, tfTransform_to_SE3f(robot2camera), tfTransform_to_SE3f(world2initial));
    
    img_count_++;
    ofstream f;
    f.open(working_path_ + "img_counter.txt", std::ios::app);
    if (f.is_open())
    {
        f << img_count_ << endl;
        f.close();
    }
    else
    {
        ROS_WARN_STREAM("Impossible to open " << working_path_ + "img_counter.txt");
    }

    timer.start();
}

void ImageGrabber::TimerCallback(const ros::WallTimerEvent& event)
{
    ROS_WARN("Writting extra files!");

    pSLAM->SaveKeyFrameTrajectoryCustom(keyframes_poses_file, tfTransform_to_SE3f(robot2camera), tfTransform_to_SE3f(world2initial));
    pSLAM->SaveLoopAndMergeEdgesCustom(loop_closure_edges_file, tfTransform_to_SE3f(robot2camera), tfTransform_to_SE3f(world2initial));
}