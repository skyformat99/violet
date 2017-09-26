/*
 *  Copyright (C) 2017 Ongun Kanat <ongun.kanat@gmail.com>
 *  Copyright (C) 2017 Arda İnceoğlu <93arda@gmail.com>
 *  Copyright (C) 2017 Istanbul Technical University
 *                     Artificial Intelligence and Robotics Laboratory
 *                     <air.cs.itu.edu.tr>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string>
#include <map>
#include <tf/transform_listener.h>
#include <violet/input_source_manager.h>
#include <violet/object_info.h>
#include <violet/knowledge_base.h>
#include <violet/util.h>

namespace violet {
namespace inputsources {
static double cloudIntersectionRate( tf::Point existingLocation, tf::Point existingSize, tf::Point newLocation, tf::Point newSize, double range_allowance = 0.005)
{
    tf::Point existingSP; // start points
    tf::Point newSP;
    tf::Point existingEP; // end points
    tf::Point newEP;

    existingSP = existingLocation - (existingSize / 2);
    newSP = newLocation - (newSize / 2);
    existingEP = existingLocation + (existingSize / 2);
    newEP = newLocation + (newSize / 2);

    bool intersectX = util::checkRangesOverlap(existingSP.x(), existingEP.x(), newSP.x(), newEP.x(), range_allowance);
    double rateX = util::rangeIntersectionRate(existingSP.x(), existingEP.x(), newSP.x(), newEP.x());

    bool intersectY = util::checkRangesOverlap(existingSP.y(), existingEP.y(), newSP.y(), newEP.y(), range_allowance);
    double rateY = util::rangeIntersectionRate(existingSP.y(), existingEP.y(), newSP.y(), newEP.y());

    bool intersectZ = util::checkRangesOverlap(existingSP.z(), existingEP.z(), newSP.z(), newEP.z(), range_allowance);
    double rateZ = util::rangeIntersectionRate(existingSP.z(), existingEP.z(), newSP.z(), newEP.z());

    return (intersectX ? rateX : 0.0) *
           (intersectY ? rateY : 0.0) *
           (intersectZ ? rateZ : 0.0);
}

class Segmentation : public InputSource
{
    ros::NodeHandle local_nh;
    const std::string REFERENCE_FRAME;
    const std::string TARGET_FRAME;

#ifdef FILTERED_SEGMENTATION
    tf::Point FILTER_START_POINT; // Filtered segmentation will create a cubic space starting from this point
    tf::Point FILTER_END_POINT;   // To this point
    ros::Publisher _filtered_segments_pub;
#endif

    tf::TransformListener _tf_listener;
    util::GaussianPdf _pdf;

public:
    Segmentation();
    virtual void callback(const violet_msgs::DetectionInfo::ConstPtr& msg);
};

Segmentation::Segmentation() :
    InputSource("Segmentation"),
    local_nh(ros::NodeHandle("~")),
    REFERENCE_FRAME(local_nh.param<std::string>("input_source_options/segmentation/reference_frame", "/camera_depth_optical_frame")),
    TARGET_FRAME(FIXED_FRAME)
{
    _pdf.sigma << 0.0005, 0,      0,
                  0,      0.0005, 0,
                  0,      0,      0.0005;

    _confidence_confusion[0][0] = 0.9;
    _confidence_confusion[0][1] = 0.1;
    _confidence_confusion[1][0] = 0.45;
    _confidence_confusion[1][1] = 0.55;
#ifdef FILTERED_SEGMENTATION
    /* Read filtering parameters from parameter server */
    double filter_start[3]; // filter start points x, y, z respectively
    double filter_end[3]; // filter end points x, y, z respectively

    filter_start[0] = local_nh.param("input_source_options/segmentation/filtering_start/x",  -0.20);
    filter_start[1] = local_nh.param("input_source_options/segmentation/filtering_start/y",  -0.35);
    filter_start[2] = local_nh.param("input_source_options/segmentation/filtering_start/z",  -0.10);

    filter_end[0] = local_nh.param("input_source_options/segmentation/filtering_end/x",  0.20);
    filter_end[1] = local_nh.param("input_source_options/segmentation/filtering_end/y",  -0.10);
    filter_end[2] = local_nh.param("input_source_options/segmentation/filtering_end/z",  0.28);

    /* This filtering code is fundementally wrong. I hate writing it. -Ongun  */
    FILTER_START_POINT.setX(filter_start[0]);
    FILTER_START_POINT.setY(filter_start[1]);
    FILTER_START_POINT.setZ(filter_start[2]);

    FILTER_END_POINT.setX(filter_end[0]);
    FILTER_END_POINT.setY(filter_end[1]);
    FILTER_END_POINT.setZ(filter_end[2]);
    /* Especially this publisher */
    _filtered_segments_pub = local_nh.advertise<violet_msgs::DetectionInfo>("filtered_segments", 100);
#endif
}

void Segmentation::callback(const violet_msgs::DetectionInfo::ConstPtr &msg)
{
    tf::Point size;
    tf::Point location;

    violet_msgs::DetectionInfo filtered_objects_msg;

    for (violet_msgs::DetectionInfo::_objects_type::const_iterator obj_it = msg->objects.begin(); obj_it != msg->objects.end(); ++obj_it) {
        for (violet_msgs::ObjectInfo::_properties_type::const_iterator prop_it = obj_it->properties.begin(); prop_it != obj_it->properties.end(); ++prop_it) {
            if(prop_it->attribute == "location") {
                location.setX(prop_it->data[0]);
                location.setY(prop_it->data[1]);
                location.setZ(prop_it->data[2]);
            }
            else if(prop_it->attribute == "size") {
                size.setX( prop_it->data[0] );
                size.setY( prop_it->data[1] );
                size.setZ( prop_it->data[2] );
            }
        }

        /* Filtering start */
        tf::Vector3 object_start = location - size * 0.5;
        tf::Vector3 object_end = location + size * 0.5;

        tf::Stamped<tf::Pose> object_start_camera(tf::Pose(tf::Quaternion(0, 0, 0, 1), object_start), msg->header.stamp, REFERENCE_FRAME);
        tf::Stamped<tf::Pose> object_start_map;
        tf::Stamped<tf::Pose> object_end_camera(tf::Pose(tf::Quaternion(0, 0, 0, 1), object_end), msg->header.stamp, REFERENCE_FRAME);
        tf::Stamped<tf::Pose> object_end_map;

        try
        {
            _tf_listener.waitForTransform( TARGET_FRAME, REFERENCE_FRAME, ros::Time(0), ros::Duration(1) );
            _tf_listener.transformPose(TARGET_FRAME, object_start_camera, object_start_map);
            _tf_listener.transformPose(TARGET_FRAME, object_end_camera, object_end_map);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN( "Transform Exception: Camera to Map, in Segmentation::callback(). Cause is: %s", ex.what() );
            continue;
        }

        object_start = object_start_map.getOrigin();
        object_end = object_end_map.getOrigin();
        tf::Point loc_pt = (object_start + object_end) * 0.5;
        /* We are always going from the small point of the map to large point of the map
         * So our size vector should always be in that direction
         */
        tf::Vector3 size_pt = (object_end - object_start).absolute();

#ifdef FILTERED_SEGMENTATION
        bool inX = util::checkRangesOverlap(object_start.x(), object_end.x(), FILTER_START_POINT.x(), FILTER_END_POINT.x(), 0.0);
        bool inY = util::checkRangesOverlap(object_start.y(), object_end.y(), FILTER_START_POINT.y(), FILTER_END_POINT.y(), 0.0);
        bool inZ = util::checkRangesOverlap(object_start.z(), object_end.z(), FILTER_START_POINT.z(), FILTER_END_POINT.z(), 0.0);

        if(!inX || !inY || !inZ) {
            continue; // Skip this object if its location is not in our field of interest
        }

        violet_msgs::ObjectInfo filtered_object;

        violet_msgs::ObjectProperty location_prop;
        location_prop.attribute = "location";
        location_prop.data.push_back(loc_pt.x());
        location_prop.data.push_back(loc_pt.y());
        location_prop.data.push_back(loc_pt.z());
        filtered_object.properties.push_back(location_prop);

        violet_msgs::ObjectProperty size_prop;
        size_prop.attribute = "size";
        size_prop.data.push_back(size_pt.x());
        size_prop.data.push_back(size_pt.y());
        size_prop.data.push_back(size_pt.z());
        filtered_object.properties.push_back(size_prop);

        filtered_objects_msg.objects.push_back(filtered_object);
#endif

        KnowledgeBase *kb = KnowledgeBase::instance();
        kb->setLastDetectionTime(_src_name, msg->header.stamp);
        const KnowledgeBase::ObjectMap& objs =  kb->objects();
        for(KnowledgeBase::ObjectMap::const_iterator it = objs.begin(); it != objs.end(); ++it ) {
            ObjectInfo* cur = it->second;
            if(cloudIntersectionRate(cur->location(), cur->size(), loc_pt, size_pt) > 0.1) {
                cur->setLastDetection(_src_name, msg->header.stamp);
                cur->increaseConfidence(_confidence_confusion);

                //TODO: Segman orta noktayı yanlış hesaplıyor gibi.
                //util::GaussianPdf update_pdf = cur->locationPDF();
                //util::updateGaussianPDF(update_pdf, object->locationPDF());
                //cur->setLocationPDF(update_pdf);
                //cur->setLocation(tf::Point(update_pdf.mu(0),  update_pdf.mu(1), update_pdf.mu(2)));
            }
        }
    }
#ifdef FILTERED_SEGMENTATION
    _filtered_segments_pub.publish(filtered_objects_msg); // After waiting an age publish the filtered objects
#endif
}

DEFINE_INPUT_SOURCE(Segmentation, Segmentation)

} /* END OF NAMESPACE inputsources */
} /* END OF NAMESPACE violet */
