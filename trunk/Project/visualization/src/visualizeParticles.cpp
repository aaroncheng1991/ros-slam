#include "../include/visualizeParticles.h"
#include "visualization_msgs/Marker.h"
#include <boost/foreach.hpp>

namespace visualization {


Visualization::Visualization(ros::NodeHandle &n) {
    this->marker_pub = n.advertise<visualization_msgs::Marker>("visualization", 10);
}


void Visualization::visualizeParticles(std::vector<fslam::Particle> particles) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "fslamparticles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    // red
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    BOOST_FOREACH(fslam::Particle particle, particles) {
        double x = particle.robotPos.position.x;
        double y = particle.robotPos.position.y;
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        marker.points.push_back(p);
    }
    marker_pub.publish(marker);
}
}

