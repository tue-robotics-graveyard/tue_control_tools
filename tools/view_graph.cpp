#include <tue_control_rtt_msgs/ControllerStates.h>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <deque>
#include <opencv2/highgui/highgui.hpp>

#include <tue/control/generic.h>

// ----------------------------------------------------------------------------------------------------

struct GraphNode
{
    GraphNode(double x_, double y_) : x(x_), y(y_) {}
    double x;
    double y;
};

// ----------------------------------------------------------------------------------------------------

class Viewer
{
public:

    Viewer() : x_min_(0), x_max_(0), y_min_(0), y_max_(0), window_(1) {}

    void setWindow(double w) { window_ = w; }

    void addPoint(int i, double x, double y)
    {
        if (!tue::control::is_set(x) || !tue::control::is_set(y))
            return;

        if (i >= data_.size())
            data_.resize(i + 1);

        std::deque<GraphNode>& d = data_[i];
        if (d.empty() || x > d.back().x)
        {
            double y_margin = 0.1 * std::abs(y);
            y_min_ = std::min(y_min_, y - y_margin);
            y_max_ = std::max(y_max_, y + y_margin);
            x_max_ = std::max(x_max_, x);
            x_min_ = x_max_ - window_;

            while(!d.empty() && d.front().x < x_min_)
                d.pop_front();
            d.push_back(GraphNode(x, y));
        }
    }

    bool graphToCanvas(const cv::Mat& canvas, double x, double y, cv::Point* p)
    {
        double x_min = x_max_ - window_;
        if (x < x_min || x > x_max_ || y < y_min_ || y > y_max_)
            return false;

        p->x = ((x - x_min) * canvas.cols) / window_;
        p->y = canvas.rows - (((y - y_min_) * canvas.rows) / (y_max_ - y_min_));

        return true;
    }

    void draw(cv::Mat& canvas)
    {
        static cv::Scalar COLORS[] = { cv::Scalar(255, 0,   0), cv::Scalar(0, 0, 255),   cv::Scalar(0, 255, 0),
                                       cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255),
                                       cv::Scalar(0, 0, 0),     cv::Scalar(128, 128, 128), cv::Scalar(100, 255, 0) };
        static int NUM_COLORS = 9;

        for(unsigned int i = 0; i < data_.size(); ++i)
        {
            const std::deque<GraphNode>& d = data_[i];
            const cv::Scalar& color = COLORS[i % NUM_COLORS];

            for(std::deque<GraphNode>::const_iterator it2 = d.begin(); it2 != d.end(); ++it2)
            {
                const GraphNode& n = *it2;
                cv::Point p;
                if (graphToCanvas(canvas, n.x, n.y, &p))
                    cv::circle(canvas, p, 1, color, CV_FILLED);
            }
        }
    }

private:

    double x_min_, x_max_;
    double y_min_, y_max_;

    double window_;

    std::vector<std::deque<GraphNode> > data_;
};


// ----------------------------------------------------------------------------------------------------

Viewer viewer;

// ----------------------------------------------------------------------------------------------------

void controllerStateCb(const tue_control_rtt_msgs::ControllerStates::ConstPtr& msg)
{
    double t = msg->timestamp.toSec();

    for(unsigned int i = 0; i < msg->states.size(); ++i)
    {
        const tue_control_rtt_msgs::ControllerState& s = msg->states[i];

        viewer.addPoint(i * 2, t, s.measurement);
        viewer.addPoint(i * 2 + 1, t, s.ref_position);
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "controller_graph_viewer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("controller_states", 1, controllerStateCb);

    cv::Mat canvas(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));

    viewer.setWindow(10);

    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();

        canvas.setTo(cv::Scalar(255, 255, 255));
        viewer.draw(canvas);

        cv::imshow("grap", canvas);
        cv::waitKey(3);

        r.sleep();
    }

    return 0;
}
