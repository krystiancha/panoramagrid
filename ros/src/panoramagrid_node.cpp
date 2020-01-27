#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <panoramagrid/grid.hpp>
#include <panoramagrid/gl/glrenderer.hpp>
#include <panoramagrid/gl/glapplication.hpp>
#include <panoramagrid/node.hpp>
#include <panoramagrid/spheremesh.hpp>
#include <panoramagrid/uvmaterial.hpp>


namespace pg = panoramagrid;

class PanoramagridRosBridge : public pg::gl::applications::GlApplication {
public:
    PanoramagridRosBridge(ros::NodeHandle nh) {
        nh.param<int>("width", width, 1920);
        nh.param<int>("height", height, 1080);

        ROS_INFO("Creating a renderer with resolution: %dx%d", width, height);
        renderer = std::make_shared<pg::gl::GlRenderer>(width, height);
        renderer->getCamera()->setFov(1.047);

        nh.param<std::string>("path", path, "");
        if (path == "") {
            throw std::runtime_error("Path to the grid archive is a required parameter");
        }

        ROS_INFO("Opening grid archive: %s", path.c_str());
        grid.open(path);
        grid.startThread();

        ROS_DEBUG("Initializing GLFW");
        pg::gl::applications::GlApplication::initGlfw();

        ROS_DEBUG("Creating a OpenGL context");
        initContext();
    }

    void parseArgs(int argc, char **argv) {};

    void run() {};

    void initContext() {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwSetErrorCallback([](int error, const char *description) {
            throw std::runtime_error(description);
        });
        auto mWindow = glfwCreateWindow(width, height, "Panoramagrid", nullptr, nullptr);
        if (mWindow == nullptr) {
            throw std::runtime_error("Failed to Create OpenGL Context");
        }
        glfwMakeContextCurrent(mWindow);

#ifdef GLAD_DEBUG
        glad_set_post_callback(openglPostCallCallback);
#endif
        if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
            throw std::runtime_error("Failed to initialize GLAD");
        }
        ROS_INFO("OpenGL %s", glGetString(GL_VERSION));

        nodes.push_back(node);
        glEnable(GL_DEPTH_TEST);
    }

    cv::Mat render(KDL::Frame frame) {
        setPosition(frame.p);
        setOrientation(frame.M);
        return render();
    }

    void setPosition(KDL::Vector position) {
        if (position == lastFrame.p) {
            return;
        }
        std::array<float, 3> rosPosition {
            static_cast<float>(position.x()),
            static_cast<float>(position.y()),
            static_cast<float>(position.z()),
        };

        auto start = std::chrono::high_resolution_clock::now();
        auto gridRes = grid.get(rosPosition);
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);

        if (abs(gridRes.first[0] - lastImage[0]) < 0.001 || abs(gridRes.first[1] - lastImage[1]) < 0.001 || abs(gridRes.first[2] - lastImage[2]) < 0.001) {
            node->getMaterial()->setTexture(gridRes.second);
            getRenderer()->render(node);
            getRenderer()->loadTexture(node->getMaterial());
            lastImage = gridRes.first;
        }

        float gain = 0.425;
        std::array<float, 3> diff {
            gain * (rosPosition[1] - gridRes.first[1]),
            0.0,
            -gain * (rosPosition[0] - gridRes.first[0]),
        };
        getRenderer()->getCamera()->setPosition(diff);
        lastFrame.p = position;
    }

    void setOrientation(KDL::Rotation orientation) {
        double roll, pitch, yaw;
        double x, y, z, w;
        orientation.GetQuaternion(x, y, z, w);
        // Quaternion transformation ROS->OpenGL
        renderer->getCamera()->setOrientation({
                                                      static_cast<float>(-y),
                                                      static_cast<float>(z),
                                                      static_cast<float>(x),
                                                      static_cast<float>(w),
                                              });
    }

    cv::Mat render() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer->render(node);
        return renderer->getMat();
    }

    pg::Grid grid;

private:
    int width, height;
    std::string path;
    std::shared_ptr<pg::Node> node = std::make_shared<pg::Node>(std::make_shared<pg::SphereMesh>(18, 36),
                                                                std::make_shared<pg::UvMaterial>());
    KDL::Frame lastFrame;
    std::array<float, 3> lastImage;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "panoramagrid_node");
    ros::NodeHandle nh("~");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    cv::Mat gazebo_img;

    std::string image_topic;
    nh.param<std::string>("image_topic", image_topic, "image");

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(image_topic, 1);
    image_transport::Subscriber sub;

    bool gazebo;
    nh.param<bool>("gazebo", gazebo, false);

    if (gazebo) {
        sub = it.subscribe("/gazebo_camera/image_raw", 1, [&](const sensor_msgs::ImageConstPtr &msg) {
            gazebo_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        });
    }

    PanoramagridRosBridge pg(nh);

    // Spawn markers
    ros::Publisher cubemapMarkerPub = nh.advertise<visualization_msgs::Marker>("cubemap_marker", 0, true);
    visualization_msgs::Marker cubemapMarker;
    cubemapMarker.header.frame_id = "world";
    cubemapMarker.header.stamp = ros::Time();
    cubemapMarker.type = visualization_msgs::Marker::POINTS;
    cubemapMarker.action = visualization_msgs::Marker::ADD;
    cubemapMarker.scale.x = 0.01;
    cubemapMarker.scale.y = 0.01;
    cubemapMarker.color.r = 0.0;
    cubemapMarker.color.g = 1.0;
    cubemapMarker.color.b = 0.0;
    cubemapMarker.color.a = 0.5;
    for (const auto &entry : pg.grid.list()) {
        geometry_msgs::Point point;
        point.x = entry[0];
        point.y = entry[1];
        point.z = entry[2];
        cubemapMarker.points.push_back(point);
    }

    std::string globalFrame, cameraFrame, gridFrame;
    nh.param<std::string>("global_frame", globalFrame, "world");
    nh.param<std::string>("camera_frame", cameraFrame, "camera");
    nh.param<std::string>("grid_frame", gridFrame, "grid");

    float fps;
    nh.param<float>("fps", fps, 60);

    ROS_INFO("Tracking transform %s -> %s, waiting for first change...", globalFrame.c_str(), cameraFrame.c_str());
    while (!tfBuffer.canTransform(globalFrame, cameraFrame, ros::Time(0)) && !tfBuffer.canTransform(globalFrame, gridFrame, ros::Time(0)) && nh.ok()) {
        ros::Duration(1 / fps).sleep();
    };

    ROS_INFO("Running with target FPS: %.2f", fps);
    ros::Rate rate(fps);
    cv::TickMeter meter;
    while (nh.ok()) {
        meter.stop();
        meter.start();

        cubemapMarkerPub.publish(cubemapMarker);

        if (meter.getCounter() > static_cast<int>(fps)) {
            ROS_INFO("%d transforms processed, avg. FPS: %.2f", static_cast<int>(fps),
                     meter.getCounter() / meter.getTimeSec());
            meter.reset();
        }

        if (gazebo && gazebo_img.empty()) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        try {
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(globalFrame, cameraFrame,
                                                                                        ros::Time(0));
            KDL::Frame frame = tf2::transformToKDL(transformStamped);

            geometry_msgs::TransformStamped gridTransformStamped = tfBuffer.lookupTransform(globalFrame, gridFrame,
                                                                                        ros::Time(0));
            KDL::Frame gridFrame = tf2::transformToKDL(gridTransformStamped);

            if (gazebo) {
                cv::Mat img = pg.render(gridFrame * frame).clone();
                cv::Mat mask;
                cv::inRange(gazebo_img, cv::Scalar(178, 178, 178), cv::Scalar(178, 178, 178), mask);
                cv::bitwise_not(mask, mask);
                gazebo_img.copyTo(img, mask);
                pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg());
            } else {
                pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pg.render(gridFrame * frame)).toImageMsg());
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform exception: %s", ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
