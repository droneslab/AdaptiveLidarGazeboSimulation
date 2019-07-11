#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "adaptive_lidar_request.pb.h"


namespace gazebo
{
typedef const boost::shared_ptr<const adaptive_lidar_msgs::msgs::AdaptiveLidarRequest> AdaptiveLidarRequestPtr;

class AdaptiveLidarWorldPlugin : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // this->adaptLidar( _parent
    //                 , 32,1,-0.46,0.46
    //                 , 32,1,-0.46,0.46
    //                 , 0.2, 1.7, 1
    //                 , 0, 0.1);

    this->adaptLidar( _parent
                    , 32,1,-0.26,0.26
                    , 32,1,-0.26,0.26
                    , 0.2, 2.7, 1
                    , 0, 0.1);

    // Listen to topic for Lidar changes
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/adaptive_lidar", &AdaptiveLidarWorldPlugin::callback, this);
  }

  public: void adaptLidar( physics::WorldPtr _parent
                         , int hSamples, double hRes, double hMinAngle, double hMaxAngle
                         , int vSamples, double vRes, double vMinAngle, double vMaxAngle
                         , double rMin, double rMax, double rRes
                         , double nMean, double nStd) {
        // Remove Lidar
        _parent->RemoveModel("adaptive_lidar_world");

        // Generate SDF
        sdf::SDF lidarSDF;
        lidarSDF.SetFromString(makeModelString( hSamples, hRes, hMinAngle, hMaxAngle
                                              , vSamples, vRes, vMinAngle, vMaxAngle
                                              , rMin, rMax, rRes
                                              , nMean, nStd));
        // Insert New Lidar
        sdf::ElementPtr model = lidarSDF.Root()->GetElement("model");
        model->GetAttribute("name")->SetFromString("adaptive_lidar_world");
        _parent->InsertModelSDF(lidarSDF);
  }

  public: void callback(AdaptiveLidarRequestPtr &msg)
  {
      // this->adaptLidar( msg->hsamples(), msg->hres(), msg->hminangle(), msg->hmaxangle()
      //                 , msg->hsamples(), msg->hres(), msg->hminangle(), msg->hmaxangle()
      //                 , msg->rmin(), msg->rmax(), msg->rres()
      //                 , msg->nmean(), msg->nstd()
      //                 );
  }

  private: std::string makeModelString( int hSamples, double hRes, double hMinAngle, double hMaxAngle
                                      , int vSamples, double vRes, double vMinAngle, double vMaxAngle
                                      , double rMin, double rMax, double rRes
                                      , double nMean, double nStd) {
    std::stringstream result;
    result << "<sdf version ='1.4'>\
         <model name='adaptive_lidar_world'>\
         <static>true</static>\
        <link name='chassis'>\
            <pose>0 0 0 0 0 0</pose>\
            <collision name='collision'>\
                <geometry>\
                    <cylinder>\
                        <radius>0.0508</radius>\
                        <length>0.0762</length>\
                    </cylinder>\
                </geometry>\
            </collision>\
            <visual name='visual'>\
                <pose>0 0 0.05 0 0 0</pose>\
                <geometry>\
                    <cylinder>\
                        <radius>0.0508</radius>\
                        <length>0.0762</length>\
                    </cylinder>\
                </geometry>\
            </visual>\
            <sensor name='fov1' type='ray'>\
                <pose>0 0 0.1 0 0 0</pose>\
                <visualize>true</visualize>\
                <ray>\
                    <scan>\
                        <horizontal>\
                            <samples>" << hSamples << "</samples>\
                            <resolution>" << hRes << "</resolution>\
                            <min_angle>" << hMinAngle << "</min_angle>\
                            <max_angle>" << hMaxAngle << "</max_angle>\
                        </horizontal>\
                        <vertical>\
                            <samples>" << vSamples << "</samples>\
                            <resolution>" << vRes << "</resolution>\
                            <min_angle>" << vMinAngle << "</min_angle>\
                            <max_angle>" << vMaxAngle << "</max_angle>\
                        </vertical>\
                    </scan>\
                    <range>\
                        <min>" << rMin << "</min>\
                        <max>" << rMax << "</max>\
                        <resolution>"<< rRes <<  "</resolution>\
                    </range>\
                    <noise>\
                        <type>gaussian</type>\
                        <mean>" << nMean << "</mean>\
                        <stddev>" << nStd <<"</stddev>\
                    </noise>\
                </ray>\
            </sensor>\
        </link>\
    </model>\
        </sdf>";
    return result.str();
  }
};


// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(AdaptiveLidarWorldPlugin)
}
