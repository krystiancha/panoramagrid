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
        nh.param<int>("width", width, 1280);
        nh.param<int>("height", height, 720);

        ROS_INFO("Creating a renderer with resolution: %dx%d", width, height);
        renderer = std::make_shared<pg::gl::GlRenderer>(width, height);

        nh.param<std::string>("path", path, "");
        if (path == "") {
            throw std::runtime_error("Path to the grid archive is a required parameter");
        }

        ROS_INFO("Opening grid archive: %s", path.c_str());
        grid.open(path);
        
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
        if (frame.p != cache.first.p || cache.second.empty()) {
            setPosition(frame.p);
            cache = std::make_pair(KDL::Frame(), cv::Mat());  // reset cache
        }
        if (frame.M != cache.first.M || cache.second.empty()) {
            ROS_DEBUG("Cache miss: frame changed, rendering...");
            setOrientation(frame.M);
            cache.first = frame;
            cache.second = render();
        }
        return cache.second;
    }
    void setPosition(KDL::Vector position) {
        cv::Mat img = grid.get({
            static_cast<float>(position.x()),
            static_cast<float>(position.y()),
            static_cast<float>(position.z()),
        }).second;
        node->getMaterial()->setTexture(img);
        getRenderer()->render(node);
        getRenderer()->loadTexture(node->getMaterial());
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
    std::pair<KDL::Frame, cv::Mat> cache;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "panoramagrid_node");
    ros::NodeHandle nh("~");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image", 1);

    PanoramagridRosBridge pg(nh);

    // Spawn markers
    ros::Publisher cubemapMarkerPub = nh.advertise<visualization_msgs::Marker>("cubemap_marker", 0, true);
    visualization_msgs::Marker cubemapMarker;
    cubemapMarker.header.frame_id = "map";
    cubemapMarker.header.stamp = ros::Time();
    cubemapMarker.type = visualization_msgs::Marker::POINTS;
    cubemapMarker.action = visualization_msgs::Marker::ADD;
    cubemapMarker.scale.x = 0.1;
    cubemapMarker.scale.y = 0.1;
    cubemapMarker.color.r = 1.0;
    cubemapMarker.color.g = 0.0;
    cubemapMarker.color.b = 0.0;
    cubemapMarker.color.a = 1.0;
    for (const auto &entry : pg.grid.list()) {
        geometry_msgs::Point point;
        point.x = entry.first[0];
        point.y = entry.first[1];
        point.z = entry.first[2];
        cubemapMarker.points.push_back(point);
    }

    std::string globalFrame, cameraFrame;
    nh.param<std::string>("global_frame", globalFrame, "map");
    nh.param<std::string>("camera_frame", cameraFrame, "camera");
    
    float fps;
    nh.param<float>("fps", fps, 60);

    ROS_INFO("Tracking transform %s -> %s, waiting for first change...", globalFrame.c_str(), cameraFrame.c_str());
    while (!tfBuffer.canTransform(globalFrame, cameraFrame, ros::Time(0)) && nh.ok()) { ros::Duration(1 / fps).sleep(); };

    ROS_INFO("Running with target FPS: %.2f", fps);
    ros::Rate rate(fps);
    cv::TickMeter meter;
    while (nh.ok()) {
        meter.stop(); meter.start();

        cubemapMarkerPub.publish(cubemapMarker);

        if (meter.getCounter() > static_cast<int>(fps)) {
            ROS_INFO("%d transforms processed, avg. FPS: %.2f", static_cast<int>(fps), meter.getCounter() / meter.getTimeSec());
            meter.reset();
        }

        try {
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(globalFrame, cameraFrame, ros::Time(0));
            KDL::Frame frame = tf2::transformToKDL(transformStamped);
            pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", pg.render(frame)).toImageMsg());
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform exception: %s", ex.what());
	    }

        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}