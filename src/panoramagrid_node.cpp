#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <panoramagrid/panoramagrid.hpp>
#include <panoramagrid/gl/gl.hpp>
#include <panoramagrid/gl/applications/equirectviewer.hpp>

namespace pg = panoramagrid;

class PanoramagridRosBridge : public pg::gl::applications::EquirectViewer {
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
        pg::gl::applications::EquirectViewer::initGlfw();

        ROS_DEBUG("Creating a OpenGL context");
        initContext();
    }
    void parseArgs(int argc, char **argv) override {};
    void initContext() override {
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
        double r, p, y;
        orientation.GetRPY(r, p, y);
        renderer->getCamera()->setOrientation({
            static_cast<float>(r), 
            static_cast<float>(p),
            static_cast<float>(y),
        });
    }
    cv::Mat render() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer->render(node);
        return renderer->getMat();
    }

private:
    int width, height;
    std::string path;
    pg::Grid grid;
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
