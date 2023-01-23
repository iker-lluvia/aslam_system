/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap_server/OctomapServer.h>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal(double a, double b, double epsilon = 1.0e-7) {
    return std::abs(a - b) < epsilon;
}

namespace octomap_server {

    OctomapServer::OctomapServer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
            : m_nh(nh_),
              m_nh_private(private_nh_),
              m_pointCloudSub(NULL),
              m_tfPointCloudSub(NULL),
              m_reconfigureServer(m_config_mutex, private_nh_),
              m_octree(NULL),
              m_maxRange(-1.0),
              m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
              m_useHeightMap(true),
              m_useColoredMap(false),
              m_useUncertaintyMap(false),
              m_colorFactor(0.8),
              m_latchedTopics(true),
              m_publishFreeSpace(false),
              m_res(0.05),
              m_treeDepth(0),
              m_maxTreeDepth(0),
              m_pointcloudMinX(-std::numeric_limits<double>::max()),
              m_pointcloudMaxX(std::numeric_limits<double>::max()),
              m_pointcloudMinY(-std::numeric_limits<double>::max()),
              m_pointcloudMaxY(std::numeric_limits<double>::max()),
              m_pointcloudMinZ(-std::numeric_limits<double>::max()),
              m_pointcloudMaxZ(std::numeric_limits<double>::max()),
              m_occupancyMinZ(-std::numeric_limits<double>::max()),
              m_occupancyMaxZ(std::numeric_limits<double>::max()),
              m_minSizeX(0.0), m_minSizeY(0.0),
              m_filterSpeckles(false), m_filterGroundPlane(false),
              m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
              m_compressMap(true),
              m_incrementalUpdate(false),
              m_initConfig(true) {
        double probHit, probMiss, thresMin, thresMax;

        m_nh_private.param("frame_id", m_worldFrameId, m_worldFrameId);
        m_nh_private.param("base_frame_id", m_baseFrameId, m_baseFrameId);
        m_nh_private.param("height_map", m_useHeightMap, m_useHeightMap);
        m_nh_private.param("colored_map", m_useColoredMap, m_useColoredMap);
        m_nh_private.param("uncertainty_map", m_useUncertaintyMap, m_useUncertaintyMap);
        m_nh_private.param("color_factor", m_colorFactor, m_colorFactor);

        m_nh_private.param("pointcloud_min_x", m_pointcloudMinX, m_pointcloudMinX);
        m_nh_private.param("pointcloud_max_x", m_pointcloudMaxX, m_pointcloudMaxX);
        m_nh_private.param("pointcloud_min_y", m_pointcloudMinY, m_pointcloudMinY);
        m_nh_private.param("pointcloud_max_y", m_pointcloudMaxY, m_pointcloudMaxY);
        m_nh_private.param("pointcloud_min_z", m_pointcloudMinZ, m_pointcloudMinZ);
        m_nh_private.param("pointcloud_max_z", m_pointcloudMaxZ, m_pointcloudMaxZ);
        m_nh_private.param("occupancy_min_z", m_occupancyMinZ, m_occupancyMinZ);
        m_nh_private.param("occupancy_max_z", m_occupancyMaxZ, m_occupancyMaxZ);
        m_nh_private.param("min_x_size", m_minSizeX, m_minSizeX);
        m_nh_private.param("min_y_size", m_minSizeY, m_minSizeY);

        m_nh_private.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
        m_nh_private.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
        // distance of points from plane for RANSAC
        m_nh_private.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
        // angular derivation of found plane:
        m_nh_private.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
        // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
        m_nh_private.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

        m_nh_private.param("sensor_model/max_range", m_maxRange, m_maxRange);

        m_nh_private.param("resolution", m_res, m_res);
        m_nh_private.param("sensor_model/hit", probHit, 0.7);
        m_nh_private.param("sensor_model/miss", probMiss, 0.4);
        m_nh_private.param("sensor_model/min", thresMin, 0.12);
        m_nh_private.param("sensor_model/max", thresMax, 0.97);
        m_nh_private.param("compress_map", m_compressMap, m_compressMap);
        m_nh_private.param("incremental_2D_projection", m_incrementalUpdate, m_incrementalUpdate);

        if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)) {
            ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
                                    << m_pointcloudMinZ << ", " << m_pointcloudMaxZ
                                    << "], excluding the ground level z=0. "
                                    << "This will not work.");
        }

        if ((m_useHeightMap && m_useColoredMap)
            || (m_useHeightMap && m_useUncertaintyMap)) {
            ROS_WARN_STREAM(
                    "You enabled both height map and RGB color or height map and uncertainty registration. This is contradictory. Defaulting to height map.");
            m_useColoredMap = false;
            m_useUncertaintyMap = false;
        }

        if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
            ROS_INFO_STREAM("Using RGB color registration (if information available)");
#elif UNCERTAINTY_OCTOMAP_SERVER
            ROS_INFO_STREAM("Using RGB color registration (if information available)");
#else
            ROS_ERROR_STREAM(
                    "Colored map requested in launch file - node not running/compiled to support colors, please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
#endif
        }

        if (m_useUncertaintyMap) {
#ifdef UNCERTAINTY_OCTOMAP_SERVER
            ROS_INFO_STREAM("Using RGB color and uncertainty registration (if information available)");
#else
            ROS_ERROR_STREAM(
                    "Uncertainty map requested in launch file - node not running/compiled to support uncertainty, please define UNCERTAINTY_OCTOMAP_SERVER and recompile or launch the octomap_uncertainty_server node");
#endif
        }

        // initialize octomap object & params
        m_octree = new OcTreeT(m_res);
        m_octree->setProbHit(probHit);
        m_octree->setProbMiss(probMiss);
        m_octree->setClampingThresMin(thresMin);
        m_octree->setClampingThresMax(thresMax);
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
        m_gridmap.info.resolution = m_res;

        double r, g, b, a;
        m_nh_private.param("color/r", r, 0.0);
        m_nh_private.param("color/g", g, 0.0);
        m_nh_private.param("color/b", b, 1.0);
        m_nh_private.param("color/a", a, 1.0);
        m_color.r = r;
        m_color.g = g;
        m_color.b = b;
        m_color.a = a;

        m_nh_private.param("color_free/r", r, 0.0);
        m_nh_private.param("color_free/g", g, 1.0);
        m_nh_private.param("color_free/b", b, 0.0);
        m_nh_private.param("color_free/a", a, 1.0);
        m_colorFree.r = r;
        m_colorFree.g = g;
        m_colorFree.b = b;
        m_colorFree.a = a;

        m_nh_private.param("publish_free_space", m_publishFreeSpace, m_publishFreeSpace);

        m_nh_private.param("latch", m_latchedTopics, m_latchedTopics);
        if (m_latchedTopics) {
            ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
        } else
            ROS_INFO(
                    "Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");

        m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, m_latchedTopics);
        m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, m_latchedTopics);
        m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1, m_latchedTopics);
        m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, m_latchedTopics);
        m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 1, m_latchedTopics);
        m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, m_latchedTopics);

        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, "cloud_in", 1);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener,
                                                                            m_worldFrameId, 1);
        m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, _1));

        m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &OctomapServer::octomapBinarySrv, this);
        m_octomapFullService = m_nh.advertiseService("octomap_full", &OctomapServer::octomapFullSrv, this);
        m_clearBBXService = m_nh_private.advertiseService("clear_bbx", &OctomapServer::clearBBXSrv, this);
        m_resetService = m_nh_private.advertiseService("reset", &OctomapServer::resetSrv, this);
#ifdef UNCERTAINTY_OCTOMAP_SERVER
        srvEstimateGain_ = m_nh_private.advertiseService("estimate_gain", &OctomapServer::estimateGainSrv, this);
        srvGetAABBoxGain_ = m_nh_private.advertiseService("get_AABBox_gain", &OctomapServer::getAABBoxGainSrv, this);
        srvGetAABBoxInfo_ = m_nh_private.advertiseService("get_AABBox_info", &OctomapServer::getAABBoxInfoSrv, this);
        srvGetMapInfo_ = m_nh_private.advertiseService("get_map_info", &OctomapServer::getMapInfoSrv, this);
        srvGetMapVoxels_ = m_nh_private.advertiseService("get_map_voxels", &OctomapServer::getMapVoxelsSrv, this);
#endif

        dynamic_reconfigure::Server<OctomapServerConfig>::CallbackType f;
        f = boost::bind(&OctomapServer::reconfigureCallback, this, _1, _2);
        m_reconfigureServer.setCallback(f);
    }

    OctomapServer::~OctomapServer() {
        if (m_tfPointCloudSub) {
            delete m_tfPointCloudSub;
            m_tfPointCloudSub = NULL;
        }

        if (m_pointCloudSub) {
            delete m_pointCloudSub;
            m_pointCloudSub = NULL;
        }


        if (m_octree) {
            delete m_octree;
            m_octree = NULL;
        }

    }


    bool OctomapServer::openFile(const std::string &filename) {
        if (filename.length() <= 3)
            return false;

        std::string suffix = filename.substr(filename.length() - 3, 3);
        if (suffix == ".bt") {
            if (!m_octree->readBinary(filename)) {
                return false;
            }
        } else if (suffix == ".ot") {
            AbstractOcTree *tree = AbstractOcTree::read(filename);
            if (!tree) {
                return false;
            }
            if (m_octree) {
                delete m_octree;
                m_octree = NULL;
            }
            m_octree = dynamic_cast<OcTreeT *>(tree);
            if (!m_octree) {
                ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
                return false;
            }

        } else {
            return false;
        }

        ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octree->size());

        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
        m_res = m_octree->getResolution();
        m_gridmap.info.resolution = m_res;
        double minX, minY, minZ;
        double maxX, maxY, maxZ;
        m_octree->getMetricMin(minX, minY, minZ);
        m_octree->getMetricMax(maxX, maxY, maxZ);

        m_updateBBXMin[0] = m_octree->coordToKey(minX);
        m_updateBBXMin[1] = m_octree->coordToKey(minY);
        m_updateBBXMin[2] = m_octree->coordToKey(minZ);

        m_updateBBXMax[0] = m_octree->coordToKey(maxX);
        m_updateBBXMax[1] = m_octree->coordToKey(maxY);
        m_updateBBXMax[2] = m_octree->coordToKey(maxZ);

        publishAll();

        return true;

    }

    void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
        ros::WallTime startTime = ros::WallTime::now();


        //
        // ground filtering in base frame
        //
        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(*cloud, pc);

        tf::StampedTransform sensorToWorldTf;
        try {
            m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
        } catch (tf::TransformException &ex) {
            ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
            return;
        }

        Eigen::Matrix4f sensorToWorld;
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);


        // set up filter for height range, also removes NANs:
        pcl::PassThrough <PCLPoint> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
        pcl::PassThrough <PCLPoint> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
        pcl::PassThrough <PCLPoint> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

        PCLPointCloud pc_ground; // segmented ground plane
        PCLPointCloud pc_nonground; // everything else

        if (m_filterGroundPlane) {
            tf::StampedTransform sensorToBaseTf, baseToWorldTf;
            try {
                m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp,
                                              ros::Duration(0.2));
                m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp,
                                             sensorToBaseTf);
                m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, baseToWorldTf);


            } catch (tf::TransformException &ex) {
                ROS_ERROR_STREAM("Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
                                                                                             "You need to set the base_frame_id or disable filter_ground.");
            }


            Eigen::Matrix4f sensorToBase, baseToWorld;
            pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
            pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

            // transform pointcloud from sensor frame to fixed robot frame
            pcl::transformPointCloud(pc, pc, sensorToBase);
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);
            filterGroundPlane(pc, pc_ground, pc_nonground);

            // transform clouds to world frame for insertion
            pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
            pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
        } else {
            // directly transform to map frame:
            pcl::transformPointCloud(pc, pc, sensorToWorld);

            // just filter height range:
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);

            pc_nonground = pc;
            // pc_nonground is empty without ground segmentation
            pc_ground.header = pc.header;
            pc_nonground.header = pc.header;
        }


        insertScan(sensorToWorldTf, pc_ground, pc_nonground);

        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)",
                  pc_ground.size(), pc_nonground.size(), total_elapsed);

        publishAll(cloud->header.stamp);
    }

    void OctomapServer::insertScan(const tf::StampedTransform &sensorToWorldTf, const PCLPointCloud &ground,
                                   const PCLPointCloud &nonground) {
        const tf::Point sensorOriginTf = sensorToWorldTf.getOrigin();
        point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
            || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
            ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
        }

#ifdef COLOR_OCTOMAP_SERVER
        unsigned char* colors = new unsigned char[3];
#endif
#ifdef UNCERTAINTY_OCTOMAP_SERVER
        unsigned char* colors = new unsigned char[3];
#endif

        // instead of direct scan insertion, compute update to filter ground:
        KeySet free_cells, occupied_cells;
        // insert ground points only as free:
        for (PCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it) {
            point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange)) {
                point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            }

            // only clear space (ground points)
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            octomap::OcTreeKey endKey;
            if (m_octree->coordToKeyChecked(point, endKey)) {
                updateMinKey(endKey, m_updateBBXMin);
                updateMaxKey(endKey, m_updateBBXMax);
            } else {
                ROS_ERROR_STREAM("Could not generate Key for endpoint " << point);
            }
        }

        // all other points: free on ray, occupied on endpoint:
        for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it) {
            point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange)) {

                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
                // occupied endpoint
                OcTreeKey key;
                if (m_octree->coordToKeyChecked(point, key)) {
                    occupied_cells.insert(key);

                    updateMinKey(key, m_updateBBXMin);
                    updateMaxKey(key, m_updateBBXMax);

#ifdef COLOR_OCTOMAP_SERVER // NB: Only read and interpret color if it's an occupied node
                    m_octree->averageNodeColor(it->x, it->y, it->z, /*r=*/it->r, /*g=*/it->g, /*b=*/it->b);
#endif

#ifdef UNCERTAINTY_OCTOMAP_SERVER // Calculate the uncertainty if it's an occupied node
                    // Color
                    m_octree->averageNodeColor(it->x, it->y, it->z, /*r=*/it->r, /*g=*/it->g, /*b=*/it->b);

                    tf::Transform pointToWorldTf;
                    pointToWorldTf.setOrigin( tf::Vector3(point.x(), point.y(), point.z()) );
                    tf::Transform pointToSensorTf = sensorToWorldTf.inverse() * pointToWorldTf;
                    float new_uncertainty = m_octree->calculateUncertainty(pointTfToOctomap(pointToSensorTf.getOrigin()));
                    m_octree->minNodeUncertainty(it->x, it->y, it->z, new_uncertainty);
#endif

                }
            } else {// ray longer than maxrange:;
                point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
                if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)) {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                    octomap::OcTreeKey endKey;
                    if (m_octree->coordToKeyChecked(new_end, endKey)) {
                        free_cells.insert(endKey);
                        updateMinKey(endKey, m_updateBBXMin);
                        updateMaxKey(endKey, m_updateBBXMax);
                    } else {
                        ROS_ERROR_STREAM("Could not generate Key for endpoint " << new_end);
                    }


                }
            }
        }

        // mark free cells only if not seen occupied in this cloud
        for (KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
            point3d point = m_octree->keyToCoord(*it);
            if (occupied_cells.find(*it) == occupied_cells.end()) {
                m_octree->updateNode(*it, false);
#ifdef UNCERTAINTY_OCTOMAP_SERVER // Set uncertainty if it's a free node
                //                m_octree->setNodeUncertainty(*it, 1.0);
                //                tf::Transform pointToWorldTf;
                //                pointToWorldTf.setOrigin( tf::Vector3(point.x(), point.y(), point.z()) );
                //                tf::Transform pointToSensorTf = sensorToWorldTf.inverse() * pointToWorldTf;
                //                float new_uncertainty = m_octree->calculateUncertainty(pointTfToOctomap(pointToSensorTf.getOrigin()));
                //                m_octree->minNodeUncertainty(*it, new_uncertainty);
#endif
            }
        }

        // now mark all occupied cells:
        for (KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
            m_octree->updateNode(*it, true);
        }

        // TODO: eval lazy+updateInner vs. proper insertion
        // non-lazy by default (updateInnerOccupancy() too slow for large maps)
        //m_octree->updateInnerOccupancy();
        octomap::point3d minPt, maxPt;
        ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " << m_updateBBXMin[1] << " "
                                                        << m_updateBBXMin[2] << " / " << m_updateBBXMax[0] << " "
                                                        << m_updateBBXMax[1] << " " << m_updateBBXMax[2]);

        // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
//   if (m_maxTreeDepth < 16)
//   {
//      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth); // this should give us the first key at depth m_maxTreeDepth that is smaller or equal to m_updateBBXMin (i.e. lower left in 2D grid coordinates)
//      OcTreeKey tmpMax = getIndexKey(m_updateBBXMax, m_maxTreeDepth); // see above, now add something to find upper right
//      tmpMax[0]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      tmpMax[1]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      tmpMax[2]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      m_updateBBXMin = tmpMin;
//      m_updateBBXMax = tmpMax;
//   }

        // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
        minPt = m_octree->keyToCoord(m_updateBBXMin);
        maxPt = m_octree->keyToCoord(m_updateBBXMax);
        ROS_DEBUG_STREAM("Updated area bounding box: " << minPt << " - " << maxPt);
        ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " << m_updateBBXMin[1] << " "
                                                       << m_updateBBXMin[2] << " / " << m_updateBBXMax[0] << " "
                                                       << m_updateBBXMax[1] << " " << m_updateBBXMax[2]);

        if (m_compressMap)
            m_octree->prune();

#ifdef COLOR_OCTOMAP_SERVER
        if (colors)
        {
          delete[] colors;
          colors = NULL;
        }
#endif
#ifdef UNCERTAINTY_OCTOMAP_SERVER
        if (colors)
        {
          delete[] colors;
          colors = NULL;
        }
#endif
    }


    void OctomapServer::publishAll(const ros::Time &rostime) {
        ros::WallTime startTime = ros::WallTime::now();
        size_t octomapSize = m_octree->size();
        // TODO: estimate num occ. voxels for size of arrays (reserve)
        if (octomapSize <= 1) {
            ROS_WARN("Nothing to publish, octree is empty");
            return;
        }

        bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
        bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
        bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
        bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
        bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
        m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

        // init markers for free space:
        visualization_msgs::MarkerArray freeNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        freeNodesVis.markers.resize(m_treeDepth + 1);

        geometry_msgs::Pose pose;
        pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

        // init markers:
        visualization_msgs::MarkerArray occupiedNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        occupiedNodesVis.markers.resize(m_treeDepth + 1);

        // init pointcloud:
        pcl::PointCloud <PCLPoint> pclCloud;

        // call pre-traversal hook:
        handlePreNodeTraversal(rostime);

        // now, traverse all leafs in the tree:
        for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
                     end = m_octree->end(); it != end; ++it) {
            bool inUpdateBBX = isInUpdateBBX(it);

            // call general hook:
            handleNode(it);
            if (inUpdateBBX)
                handleNodeInBBX(it);

            if (m_octree->isNodeOccupied(*it)) {
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
                    double size = it.getSize();
                    double x = it.getX();
                    double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
                    int r = it->getColor().r;
                    int g = it->getColor().g;
                    int b = it->getColor().b;
#endif

#ifdef UNCERTAINTY_OCTOMAP_SERVER
                    // Color
                    int r = it->getColor().r;
                    int g = it->getColor().g;
                    int b = it->getColor().b;

                    float uncertainty = it->getUncertainty();
#endif

                    // Ignore speckles in the map:
                    if (m_filterSpeckles && (it.getDepth() == m_treeDepth + 1) && isSpeckleNode(it.getKey())) {
                        ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
                        continue;
                    } // else: current octree node is no speckle, send it out

                    handleOccupiedNode(it);
                    if (inUpdateBBX)
                        handleOccupiedNodeInBBX(it);


                    //create marker:
                    if (publishMarkerArray) {
                        unsigned idx = it.getDepth();
                        assert(idx < occupiedNodesVis.markers.size());

                        geometry_msgs::Point cubeCenter;
                        cubeCenter.x = x;
                        cubeCenter.y = y;
                        cubeCenter.z = z;

                        occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
                        if (m_useHeightMap) {
                            double minX, minY, minZ, maxX, maxY, maxZ;
                            m_octree->getMetricMin(minX, minY, minZ);
                            m_octree->getMetricMax(maxX, maxY, maxZ);

                            double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) *
                                       m_colorFactor;
                            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
                        }

#ifdef COLOR_OCTOMAP_SERVER
                        if (m_useColoredMap) {
                          std_msgs::ColorRGBA _color; _color.r = (r / 255.); _color.g = (g / 255.); _color.b = (b / 255.); _color.a = 1.0; // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
                          occupiedNodesVis.markers[idx].colors.push_back(_color);
                        }
#endif

#ifdef UNCERTAINTY_OCTOMAP_SERVER
                        // Color
              //          if (m_useColoredMap) {
              //            std_msgs::ColorRGBA _color; _color.r = (r / 255.); _color.g = (g / 255.); _color.b = (b / 255.); _color.a = 1.0; // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
              //            occupiedNodesVis.markers[idx].colors.push_back(_color);
              //          }

                        if (m_useUncertaintyMap) {
                          occupiedNodesVis.markers[idx].colors.push_back(uncertaintyMapColor(uncertainty));
                        }
#endif
                    }

                    // insert into pointcloud:
                    if (publishPointCloud) {
#ifdef COLOR_OCTOMAP_SERVER
                        PCLPoint _point = PCLPoint();
                        _point.x = x; _point.y = y; _point.z = z;
                        _point.r = r; _point.g = g; _point.b = b;
                        pclCloud.push_back(_point);
#elif UNCERTAINTY_OCTOMAP_SERVER
                        PCLPoint _point = PCLPoint();
                        _point.x = x; _point.y = y; _point.z = z;
                        _point.r = r; _point.g = g; _point.b = b;
                        pclCloud.push_back(_point);
#else
                        pclCloud.push_back(PCLPoint(x, y, z));
#endif
                    }

                }
            } else { // node not occupied => mark as free in 2D map if unknown so far
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
                    handleFreeNode(it);
                    if (inUpdateBBX)
                        handleFreeNodeInBBX(it);

                    if (m_publishFreeSpace) {
                        double x = it.getX();
                        double y = it.getY();

                        //create marker for free space:
                        if (publishFreeMarkerArray) {
                            unsigned idx = it.getDepth();
                            assert(idx < freeNodesVis.markers.size());

                            geometry_msgs::Point cubeCenter;
                            cubeCenter.x = x;
                            cubeCenter.y = y;
                            cubeCenter.z = z;

                            freeNodesVis.markers[idx].points.push_back(cubeCenter);
                        }
                    }

                }
            }
        }

        // call post-traversal hook:
        handlePostNodeTraversal(rostime);

        // finish MarkerArray:
        if (publishMarkerArray) {
            for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) {
                double size = m_octree->getNodeSize(i);

                occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
                occupiedNodesVis.markers[i].header.stamp = rostime;
                occupiedNodesVis.markers[i].ns = "map";
                occupiedNodesVis.markers[i].id = i;
                occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                occupiedNodesVis.markers[i].scale.x = size;
                occupiedNodesVis.markers[i].scale.y = size;
                occupiedNodesVis.markers[i].scale.z = size;
                occupiedNodesVis.markers[i].pose.orientation.x = 0;
                occupiedNodesVis.markers[i].pose.orientation.y = 0;
                occupiedNodesVis.markers[i].pose.orientation.z = 0;
                occupiedNodesVis.markers[i].pose.orientation.w = 1;
                if (!m_useColoredMap && !m_useUncertaintyMap)
                    occupiedNodesVis.markers[i].color = m_color;


                if (occupiedNodesVis.markers[i].points.size() > 0)
                    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
                else
                    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
            }

            m_markerPub.publish(occupiedNodesVis);
        }


        // finish FreeMarkerArray:
        if (publishFreeMarkerArray) {
            for (unsigned i = 0; i < freeNodesVis.markers.size(); ++i) {
                double size = m_octree->getNodeSize(i);

                freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
                freeNodesVis.markers[i].header.stamp = rostime;
                freeNodesVis.markers[i].ns = "map";
                freeNodesVis.markers[i].id = i;
                freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                freeNodesVis.markers[i].scale.x = size;
                freeNodesVis.markers[i].scale.y = size;
                freeNodesVis.markers[i].scale.z = size;
                freeNodesVis.markers[i].color = m_colorFree;
                freeNodesVis.markers[i].pose.orientation.x = 0;
                freeNodesVis.markers[i].pose.orientation.y = 0;
                freeNodesVis.markers[i].pose.orientation.z = 0;
                freeNodesVis.markers[i].pose.orientation.w = 1;

                if (freeNodesVis.markers[i].points.size() > 0)
                    freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
                else
                    freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
            }

            m_fmarkerPub.publish(freeNodesVis);
        }


        // finish pointcloud:
        if (publishPointCloud) {
            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(pclCloud, cloud);
            cloud.header.frame_id = m_worldFrameId;
            cloud.header.stamp = rostime;
            m_pointCloudPub.publish(cloud);
        }

        if (publishBinaryMap)
            publishBinaryOctoMap(rostime);

        if (publishFullMap)
            publishFullOctoMap(rostime);


        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

    }


    bool OctomapServer::octomapBinarySrv(OctomapSrv::Request &req,
                                         OctomapSrv::Response &res) {
        ros::WallTime startTime = ros::WallTime::now();
        ROS_INFO("Sending binary map data on service request");
        res.map.header.frame_id = m_worldFrameId;
        res.map.header.stamp = ros::Time::now();
        if (!octomap_msgs::binaryMapToMsg(*m_octree, res.map))
            return false;

        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
        return true;
    }

    bool OctomapServer::octomapFullSrv(OctomapSrv::Request &req,
                                       OctomapSrv::Response &res) {
        ROS_INFO("Sending full map data on service request");
        res.map.header.frame_id = m_worldFrameId;
        res.map.header.stamp = ros::Time::now();


        if (!octomap_msgs::fullMapToMsg(*m_octree, res.map))
            return false;

        return true;
    }

    bool OctomapServer::clearBBXSrv(BBXSrv::Request &req, BBXSrv::Response &resp) {
        point3d min = pointMsgToOctomap(req.min);
        point3d max = pointMsgToOctomap(req.max);

        double thresMin = m_octree->getClampingThresMin();
        for (OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min, max),
                     end = m_octree->end_leafs_bbx(); it != end; ++it) {

            it->setLogOdds(octomap::logodds(thresMin));
            //  m_octree->updateNode(it.getKey(), -6.0f);
        }
        // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
        m_octree->updateInnerOccupancy();

        publishAll(ros::Time::now());

        return true;
    }

    bool OctomapServer::resetSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
        visualization_msgs::MarkerArray occupiedNodesVis;
        occupiedNodesVis.markers.resize(m_treeDepth + 1);
        ros::Time rostime = ros::Time::now();
        m_octree->clear();
        // clear 2D map:
        m_gridmap.data.clear();
        m_gridmap.info.height = 0.0;
        m_gridmap.info.width = 0.0;
        m_gridmap.info.resolution = 0.0;
        m_gridmap.info.origin.position.x = 0.0;
        m_gridmap.info.origin.position.y = 0.0;

        ROS_INFO("Cleared octomap");
        publishAll(rostime);

        publishBinaryOctoMap(rostime);
        for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) {

            occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
            occupiedNodesVis.markers[i].header.stamp = rostime;
            occupiedNodesVis.markers[i].ns = "map";
            occupiedNodesVis.markers[i].id = i;
            occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        m_markerPub.publish(occupiedNodesVis);

        visualization_msgs::MarkerArray freeNodesVis;
        freeNodesVis.markers.resize(m_treeDepth + 1);

        for (unsigned i = 0; i < freeNodesVis.markers.size(); ++i) {

            freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
            freeNodesVis.markers[i].header.stamp = rostime;
            freeNodesVis.markers[i].ns = "map";
            freeNodesVis.markers[i].id = i;
            freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        m_fmarkerPub.publish(freeNodesVis);

        return true;
    }

#ifdef UNCERTAINTY_OCTOMAP_SERVER

    bool OctomapServer::estimateGainSrv(octomap_msgs::EstimateGain::Request& req, octomap_msgs::EstimateGain::Response& res){
    //  # EstimateGain.srv
    //  geometry_msgs/PoseStamped pose
    //  float32 h_res
    //  float32 v_res
    //  float32 angle_h_min
    //  float32 angle_h_max
    //  float32 angle_v_min
    //  float32 angle_v_max
    //  float32 range_min
    //  float32 range_max
    //  float32 unknown_gain_value
    //  ---
    //  float32 gain
    //  int unknown_cells
      float gain = 0.0;
      int unknown_cells = 0;

      tf::Transform tf_world_sensor;
      tf_world_sensor.setOrigin(tf::Vector3(req.pose.pose.position.x, req.pose.pose.position.y, req.pose.pose.position.z));
      tf_world_sensor.setRotation(tf::Quaternion(req.pose.pose.orientation.x, req.pose.pose.orientation.y, req.pose.pose.orientation.z, req.pose.pose.orientation.w));
      ROS_WARN_STREAM("x: " << tf_world_sensor.getOrigin().x() << "\ty: " << tf_world_sensor.getOrigin().y() << "\tz: " << tf_world_sensor.getOrigin().z());

      float v_step =  (req.angle_v_max - req.angle_v_min) / (req.v_res - 1);
      float h_step =  (req.angle_h_max - req.angle_h_min) / (req.h_res - 1);
      // Use ray-casting to calculate the voxels that would be seen
      for (int i = 0; i < req.v_res; ++i)  // -y
      {
        float angle_v = req.angle_v_min + (float) (v_step * i);
        for (int j = 0; j < req.h_res; ++j)  // x
        {
          float angle_h = req.angle_h_min + (float) (h_step * j);
          float u_gain = 0.0;
          // Define the straight line in which the first voxel would be seen
          int k = 0;
          float range = (float) req.range_min;
          while (range < (float) req.range_max)  // z
          {
            float ray_x = (float) (range * tan(angle_h));
            float ray_y = (float) -(range * tan(angle_v));
            float ray_z = range;

            // Transform ray_x, ray_y and ray_z from sensor frame to global frame
            tf::Transform tf_sensor_rayPoint;
            tf_sensor_rayPoint.setOrigin(tf::Vector3(ray_x, ray_y, ray_z));

            tf::Transform tf_world_rayPoint = tf_world_sensor * tf_sensor_rayPoint;

            OcTreeKey key;
            if (m_octree->coordToKeyChecked(tf_world_rayPoint.getOrigin().x(), tf_world_rayPoint.getOrigin().y(), tf_world_rayPoint.getOrigin().z(), key))
            {
              OcTreeNode* node = m_octree->search(key);
              if (node)  // The node is already seen
              {
                if (m_octree->isNodeOccupied(node))  // The node is occupied, calculate gain and break
                {
                  float u_old = m_octree->getNodeUncertainty(tf_world_rayPoint.getOrigin().x(), tf_world_rayPoint.getOrigin().y(), tf_world_rayPoint.getOrigin().z());
                  float u_new = m_octree->calculateUncertainty(pointTfToOctomap(tf_sensor_rayPoint.getOrigin()));
                  if (u_new < u_old)
                  {
                    u_gain = u_old - u_new;
                  }
                  else
                  {
                    u_gain = 0.0;
                  }
                  break;
                }
                else  // The node is free, calculate gain and continue
                {
//                  float u_old = m_octree->getNodeUncertainty(tf_world_rayPoint.getOrigin().x(), tf_world_rayPoint.getOrigin().y(), tf_world_rayPoint.getOrigin().z());
//                  float u_new = m_octree->calculateUncertainty(pointTfToOctomap(tf_sensor_rayPoint.getOrigin()));
//                  if (u_new < u_old)
//                  {
//                    u_gain = u_old - u_new;
//                  }
//                  else
//                  {
//                    u_gain = 0.0;
//                  }
                  ++k;
                  range = (float) req.range_min + m_res * k;
                }
              }
              else  // Unknown node, calculate gain and break
              {
                ++unknown_cells;
                float u_new = m_octree->calculateUncertainty(pointTfToOctomap(tf_sensor_rayPoint.getOrigin()));
                u_gain = 1.0 - u_new;
                break;
              }
            }
            else  // No key, continue
            {
              ++k;
              range = (float) req.range_min + m_res * k;
            }
          }
          gain += u_gain;
        }
      }
      ROS_WARN_STREAM("gain: " << gain << "\tunknown_cells: " << unknown_cells);
      res.gain = gain;
      res.unknown_cells = unknown_cells;
      return true;
    }

    bool OctomapServer::getAABBoxGainSrv(octomap_msgs::GetAABBoxValue::Request& req, octomap_msgs::GetAABBoxValue::Response& res) {
      point3d min = pointMsgToOctomap(req.min);
      point3d max = pointMsgToOctomap(req.max);

      float gain = 0.0;
      for(OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min,max),
          end=m_octree->end_leafs_bbx(); it!= end; ++it){
        // TODO
      }
      res.value = gain;
      return true;
    }

    bool OctomapServer::getAABBoxInfoSrv(octomap_msgs::GetAABBoxValue::Request& req, octomap_msgs::GetAABBoxValue::Response& res) {
      point3d min = pointMsgToOctomap(req.min);
      point3d max = pointMsgToOctomap(req.max);

      float info = 0.0;
      for(OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min,max),
          end=m_octree->end_leafs_bbx(); it!= end; ++it){
        info += (1 - it->getUncertainty());
      }
      res.value = info;
      return true;
    }

    bool OctomapServer::getMapInfoSrv(octomap_msgs::GetValue::Request& req, octomap_msgs::GetValue::Response& res) {
      float info = 0.0;
      float info_occupied = 0.0;
      for(OcTreeT::tree_iterator it = m_octree->begin_tree(),
           end=m_octree->end_tree(); it!= end; ++it)
      {
        info += (1 - it->getUncertainty());
        if (m_octree->isNodeOccupied(*it))
        {
          info_occupied += (1 - it->getUncertainty());
        }
      }
      res.value = info;
      res.value_occupied = info_occupied;
      return true;
    }

    bool OctomapServer::getMapVoxelsSrv(octomap_msgs::GetValue::Request& req, octomap_msgs::GetValue::Response& res) {
      int voxels = 0;
      int voxels_occupied = 0;
      for(OcTreeT::tree_iterator it = m_octree->begin_tree(),
           end=m_octree->end_tree(); it!= end; ++it)
      {
        if (it->getUncertainty() < 1)
        {
          voxels += 1;
          if (m_octree->isNodeOccupied(*it))
          {
            voxels_occupied += 1;
          }

        }
      }
      res.value = voxels;
      res.value_occupied = voxels_occupied;
      return true;
    }

#endif

    void OctomapServer::publishBinaryOctoMap(const ros::Time &rostime) const {

        Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::binaryMapToMsg(*m_octree, map))
            m_binaryMapPub.publish(map);
        else
            ROS_ERROR("Error serializing OctoMap");
    }

    void OctomapServer::publishFullOctoMap(const ros::Time &rostime) const {

        Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::fullMapToMsg(*m_octree, map))
            m_fullMapPub.publish(map);
        else
            ROS_ERROR("Error serializing OctoMap");

    }


    void
    OctomapServer::filterGroundPlane(const PCLPointCloud &pc, PCLPointCloud &ground, PCLPointCloud &nonground) const {
        ground.header = pc.header;
        nonground.header = pc.header;

        if (pc.size() < 50) {
            ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
            nonground = pc;
        } else {
            // plane detection for ground plane removal:
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            // Create the segmentation object and set up:
            pcl::SACSegmentation <PCLPoint> seg;
            seg.setOptimizeCoefficients(true);
            // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(200);
            seg.setDistanceThreshold(m_groundFilterDistance);
            seg.setAxis(Eigen::Vector3f(0, 0, 1));
            seg.setEpsAngle(m_groundFilterAngle);


            PCLPointCloud cloud_filtered(pc);
            // Create the filtering object
            pcl::ExtractIndices <PCLPoint> extract;
            bool groundPlaneFound = false;

            while (cloud_filtered.size() > 10 && !groundPlaneFound) {
                seg.setInputCloud(cloud_filtered.makeShared());
                seg.segment(*inliers, *coefficients);
                if (inliers->indices.size() == 0) {
                    ROS_INFO("PCL segmentation did not find any plane.");

                    break;
                }

                extract.setInputCloud(cloud_filtered.makeShared());
                extract.setIndices(inliers);

                if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance) {
                    ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
                              cloud_filtered.size(),
                              coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2),
                              coefficients->values.at(3));
                    extract.setNegative(false);
                    extract.filter(ground);

                    // remove ground points from full pointcloud:
                    // workaround for PCL bug:
                    if (inliers->indices.size() != cloud_filtered.size()) {
                        extract.setNegative(true);
                        PCLPointCloud cloud_out;
                        extract.filter(cloud_out);
                        nonground += cloud_out;
                        cloud_filtered = cloud_out;
                    }

                    groundPlaneFound = true;
                } else {
                    ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f",
                              inliers->indices.size(), cloud_filtered.size(),
                              coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2),
                              coefficients->values.at(3));
                    pcl::PointCloud <PCLPoint> cloud_out;
                    extract.setNegative(false);
                    extract.filter(cloud_out);
                    nonground += cloud_out;
                    // debug
                    //            pcl::PCDWriter writer;
                    //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

                    // remove current plane from scan for next iteration:
                    // workaround for PCL bug:
                    if (inliers->indices.size() != cloud_filtered.size()) {
                        extract.setNegative(true);
                        cloud_out.points.clear();
                        extract.filter(cloud_out);
                        cloud_filtered = cloud_out;
                    } else {
                        cloud_filtered.points.clear();
                    }
                }

            }
            // TODO: also do this if overall starting pointcloud too small?
            if (!groundPlaneFound) { // no plane found or remaining points too small
                ROS_WARN("No ground plane found in scan");

                // do a rough fitlering on height to prevent spurious obstacles
                pcl::PassThrough <PCLPoint> second_pass;
                second_pass.setFilterFieldName("z");
                second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
                second_pass.setInputCloud(pc.makeShared());
                second_pass.filter(ground);

                second_pass.setFilterLimitsNegative(true);
                second_pass.filter(nonground);
            }

            // debug:
            //        pcl::PCDWriter writer;
            //        if (pc_ground.size() > 0)
            //          writer.write<PCLPoint>("ground.pcd",pc_ground, false);
            //        if (pc_nonground.size() > 0)
            //          writer.write<PCLPoint>("nonground.pcd",pc_nonground, false);

        }


    }

    void OctomapServer::handlePreNodeTraversal(const ros::Time &rostime) {
        if (m_publish2DMap) {
            // init projected 2D map:
            m_gridmap.header.frame_id = m_worldFrameId;
            m_gridmap.header.stamp = rostime;
            nav_msgs::MapMetaData oldMapInfo = m_gridmap.info;

            // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            octomap::point3d minPt(minX, minY, minZ);
            octomap::point3d maxPt(maxX, maxY, maxZ);
            octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
            octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

            ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1],
                      maxKey[2]);

            // add padding if requested (= new min/maxPts in x&y):
            double halfPaddedX = 0.5 * m_minSizeX;
            double halfPaddedY = 0.5 * m_minSizeY;
            minX = std::min(minX, -halfPaddedX);
            maxX = std::max(maxX, halfPaddedX);
            minY = std::min(minY, -halfPaddedY);
            maxY = std::max(maxY, halfPaddedY);
            minPt = octomap::point3d(minX, minY, minZ);
            maxPt = octomap::point3d(maxX, maxY, maxZ);

            OcTreeKey paddedMaxKey;
            if (!m_octree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)) {
                ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
                return;
            }
            if (!m_octree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)) {
                ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
                return;
            }

            ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1],
                      m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
            assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

            m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
            m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0]) / m_multires2DScale + 1;
            m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1]) / m_multires2DScale + 1;

            int mapOriginX = minKey[0] - m_paddedMinKey[0];
            int mapOriginY = minKey[1] - m_paddedMinKey[1];
            assert(mapOriginX >= 0 && mapOriginY >= 0);

            // might not exactly be min / max of octree:
            octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
            double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
            m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes - m_gridmap.info.resolution) > 1e-6));
            m_gridmap.info.resolution = gridRes;
            m_gridmap.info.origin.position.x = origin.x() - gridRes * 0.5;
            m_gridmap.info.origin.position.y = origin.y() - gridRes * 0.5;
            if (m_maxTreeDepth != m_treeDepth) {
                m_gridmap.info.origin.position.x -= m_res / 2.0;
                m_gridmap.info.origin.position.y -= m_res / 2.0;
            }

            // workaround for  multires. projection not working properly for inner nodes:
            // force re-building complete map
            if (m_maxTreeDepth < m_treeDepth)
                m_projectCompleteMap = true;


            if (m_projectCompleteMap) {
                ROS_DEBUG("Rebuilding complete 2D map");
                m_gridmap.data.clear();
                // init to unknown:
                m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

            } else {

                if (mapChanged(oldMapInfo, m_gridmap.info)) {
                    ROS_DEBUG("2D grid map size changed to %dx%d", m_gridmap.info.width, m_gridmap.info.height);
                    adjustMapData(m_gridmap, oldMapInfo);
                }
                nav_msgs::OccupancyGrid::_data_type::iterator startIt;
                size_t mapUpdateBBXMinX = std::max(0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0])) /
                                                      int(m_multires2DScale));
                size_t mapUpdateBBXMinY = std::max(0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1])) /
                                                      int(m_multires2DScale));
                size_t mapUpdateBBXMaxX = std::min(int(m_gridmap.info.width - 1),
                                                   (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0])) /
                                                   int(m_multires2DScale));
                size_t mapUpdateBBXMaxY = std::min(int(m_gridmap.info.height - 1),
                                                   (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1])) /
                                                   int(m_multires2DScale));

                assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
                assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

                size_t numCols = mapUpdateBBXMaxX - mapUpdateBBXMinX + 1;

                // test for max idx:
                uint max_idx = m_gridmap.info.width * mapUpdateBBXMaxY + mapUpdateBBXMaxX;
                if (max_idx >= m_gridmap.data.size())
                    ROS_ERROR(
                            "BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]",
                            max_idx, m_gridmap.data.size(), m_gridmap.info.width, m_gridmap.info.height,
                            mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);

                // reset proj. 2D map in bounding box:
                for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j) {
                    std::fill_n(m_gridmap.data.begin() + m_gridmap.info.width * j + mapUpdateBBXMinX,
                                numCols, -1);
                }

            }


        }

    }

    void OctomapServer::handlePostNodeTraversal(const ros::Time &rostime) {

        if (m_publish2DMap)
            m_mapPub.publish(m_gridmap);
    }

    void OctomapServer::handleOccupiedNode(const OcTreeT::iterator &it) {

        if (m_publish2DMap && m_projectCompleteMap) {
            update2DMap(it, true);
        }
    }

    void OctomapServer::handleFreeNode(const OcTreeT::iterator &it) {

        if (m_publish2DMap && m_projectCompleteMap) {
            update2DMap(it, false);
        }
    }

    void OctomapServer::handleOccupiedNodeInBBX(const OcTreeT::iterator &it) {

        if (m_publish2DMap && !m_projectCompleteMap) {
            update2DMap(it, true);
        }
    }

    void OctomapServer::handleFreeNodeInBBX(const OcTreeT::iterator &it) {

        if (m_publish2DMap && !m_projectCompleteMap) {
            update2DMap(it, false);
        }
    }

    void OctomapServer::update2DMap(const OcTreeT::iterator &it, bool occupied) {

        // update 2D map (occupied always overrides):

        if (it.getDepth() == m_maxTreeDepth) {
            unsigned idx = mapIdx(it.getKey());
            if (occupied)
                m_gridmap.data[mapIdx(it.getKey())] = 100;
            else if (m_gridmap.data[idx] == -1) {
                m_gridmap.data[idx] = 0;
            }

        } else {
            int intSize = 1 << (m_maxTreeDepth - it.getDepth());
            octomap::OcTreeKey minKey = it.getIndexKey();
            for (int dx = 0; dx < intSize; dx++) {
                int i = (minKey[0] + dx - m_paddedMinKey[0]) / m_multires2DScale;
                for (int dy = 0; dy < intSize; dy++) {
                    unsigned idx = mapIdx(i, (minKey[1] + dy - m_paddedMinKey[1]) / m_multires2DScale);
                    if (occupied)
                        m_gridmap.data[idx] = 100;
                    else if (m_gridmap.data[idx] == -1) {
                        m_gridmap.data[idx] = 0;
                    }
                }
            }
        }


    }


    bool OctomapServer::isSpeckleNode(const OcTreeKey &nKey) const {
        OcTreeKey key;
        bool neighborFound = false;
        for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]) {
            for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]) {
                for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]) {
                    if (key != nKey) {
                        OcTreeNode *node = m_octree->search(key);
                        if (node && m_octree->isNodeOccupied(node)) {
                            // we have a neighbor => break!
                            neighborFound = true;
                        }
                    }
                }
            }
        }

        return neighborFound;
    }

    void OctomapServer::reconfigureCallback(octomap_server::OctomapServerConfig &config, uint32_t level) {
        if (m_maxTreeDepth != unsigned(config.max_depth))
            m_maxTreeDepth = unsigned(config.max_depth);
        else {
            m_pointcloudMinZ = config.pointcloud_min_z;
            m_pointcloudMaxZ = config.pointcloud_max_z;
            m_occupancyMinZ = config.occupancy_min_z;
            m_occupancyMaxZ = config.occupancy_max_z;
            m_filterSpeckles = config.filter_speckles;
            m_filterGroundPlane = config.filter_ground;
            m_compressMap = config.compress_map;
            m_incrementalUpdate = config.incremental_2D_projection;

            // Parameters with a namespace require an special treatment at the beginning, as dynamic reconfigure
            // will overwrite them because the server is not able to match parameters' names.
            if (m_initConfig) {
                // If parameters do not have the default value, dynamic reconfigure server should be updated.
                if (!is_equal(m_groundFilterDistance, 0.04))
                    config.ground_filter_distance = m_groundFilterDistance;
                if (!is_equal(m_groundFilterAngle, 0.15))
                    config.ground_filter_angle = m_groundFilterAngle;
                if (!is_equal(m_groundFilterPlaneDistance, 0.07))
                    config.ground_filter_plane_distance = m_groundFilterPlaneDistance;
                if (!is_equal(m_maxRange, -1.0))
                    config.sensor_model_max_range = m_maxRange;
                if (!is_equal(m_octree->getProbHit(), 0.7))
                    config.sensor_model_hit = m_octree->getProbHit();
                if (!is_equal(m_octree->getProbMiss(), 0.4))
                    config.sensor_model_miss = m_octree->getProbMiss();
                if (!is_equal(m_octree->getClampingThresMin(), 0.12))
                    config.sensor_model_min = m_octree->getClampingThresMin();
                if (!is_equal(m_octree->getClampingThresMax(), 0.97))
                    config.sensor_model_max = m_octree->getClampingThresMax();
                m_initConfig = false;

                boost::recursive_mutex::scoped_lock reconf_lock(m_config_mutex);
                m_reconfigureServer.updateConfig(config);
            } else {
                m_groundFilterDistance = config.ground_filter_distance;
                m_groundFilterAngle = config.ground_filter_angle;
                m_groundFilterPlaneDistance = config.ground_filter_plane_distance;
                m_maxRange = config.sensor_model_max_range;
                m_octree->setClampingThresMin(config.sensor_model_min);
                m_octree->setClampingThresMax(config.sensor_model_max);

                // Checking values that might create unexpected behaviors.
                if (is_equal(config.sensor_model_hit, 1.0))
                    config.sensor_model_hit -= 1.0e-6;
                m_octree->setProbHit(config.sensor_model_hit);
                if (is_equal(config.sensor_model_miss, 0.0))
                    config.sensor_model_miss += 1.0e-6;
                m_octree->setProbMiss(config.sensor_model_miss);
            }
        }
        publishAll();
    }

    void OctomapServer::adjustMapData(nav_msgs::OccupancyGrid &map, const nav_msgs::MapMetaData &oldMapInfo) const {
        if (map.info.resolution != oldMapInfo.resolution) {
            ROS_ERROR("Resolution of map changed, cannot be adjusted");
            return;
        }

        int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x) / map.info.resolution + 0.5);
        int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y) / map.info.resolution + 0.5);

        if (i_off < 0 || j_off < 0
            || oldMapInfo.width + i_off > map.info.width
            || oldMapInfo.height + j_off > map.info.height) {
            ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
            return;
        }

        nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

        map.data.clear();
        // init to unknown:
        map.data.resize(map.info.width * map.info.height, -1);

        nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

        for (int j = 0; j < int(oldMapInfo.height); ++j) {
            // copy chunks, row by row:
            fromStart = oldMapData.begin() + j * oldMapInfo.width;
            fromEnd = fromStart + oldMapInfo.width;
            toStart = map.data.begin() + ((j + j_off) * m_gridmap.info.width + i_off);
            copy(fromStart, fromEnd, toStart);

//    for (int i =0; i < int(oldMapInfo.width); ++i){
//      map.data[m_gridmap.info.width*(j+j_off) +i+i_off] = oldMapData[oldMapInfo.width*j +i];
//    }

        }

    }


    std_msgs::ColorRGBA OctomapServer::heightMapColor(double h) {

        std_msgs::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
            f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
            case 6:
            case 0:
                color.r = v;
                color.g = n;
                color.b = m;
                break;
            case 1:
                color.r = n;
                color.g = v;
                color.b = m;
                break;
            case 2:
                color.r = m;
                color.g = v;
                color.b = n;
                break;
            case 3:
                color.r = m;
                color.g = n;
                color.b = v;
                break;
            case 4:
                color.r = n;
                color.g = m;
                color.b = v;
                break;
            case 5:
                color.r = v;
                color.g = m;
                color.b = n;
                break;
            default:
                color.r = 1;
                color.g = 0.5;
                color.b = 0.5;
                break;
        }

        return color;
    }

    std_msgs::ColorRGBA OctomapServer::uncertaintyMapColor(float u) {

        std_msgs::ColorRGBA color;
        color.a = 1.0;

        float r = 0.0;
        float g = 1.0;
        float b = 0.0;

        // 0 --> (0, 1, 0)
        // 0.25 --> (0, 1, 1)
        // 0.5 --> (0, 0, 1)
        // 0.75 --> (1, 0, 1)
        // 0.1 --> (1, 0, 0)

        float c = u * 4;
        int f = floor(c);
        float m = c - f;

        switch (f) {
            case 0:
                color.r = 0;
                color.g = 1;
                color.b = m;
                break;
            case 1:
                color.r = 0;
                color.g = 1 - m;
                color.b = 1;
                break;
            case 2:
                color.r = m;
                color.g = 0;
                color.b = 1;
                break;
            case 3:
                color.r = 1;
                color.g = 0;
                color.b = 1 - m;
                break;
            case 4:
                color.r = 1 - m;
                color.g = 0;
                color.b = 0;
                break;
            default:
                color.r = 1;
                color.g = 1;
                color.b = 1;
                break;
        }

        return color;
    }
}



