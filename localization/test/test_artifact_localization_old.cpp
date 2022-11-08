/**
 *  @brief Old test cases for artifact_localization
 *
 *  This file contains old tests for artifact_localization that are no
 *  longer functional.  Preserved here for posterity.
 */

// TEST_F(ArtifactLocalizationFixtureTest, testGetBoundingBoxCentroid) {
//   int xmin = 30;
//   int ymin = 20;
//   int xmax = 50;
//   int ymax = 40;

//   cv::Rect bbox(xmin, ymin, xmax - xmin, ymax - ymin);
//   cv::Point2i c = c_->getBoundingBoxCentroid(bbox);
//   EXPECT_EQ(c.x, (xmin + xmax) / 2);
//   EXPECT_EQ(c.y, (ymin + ymax) / 2);
// }

// TEST_F(ArtifactLocalizationFixtureTest, testEigenTransformFromTf) {
//   static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//   geometry_msgs::TransformStamped static_transformStamped;

//   static_transformStamped.header.stamp = ros::Time::now();
//   static_transformStamped.header.frame_id = robot_namespace_frames + "/map";
//   static_transformStamped.child_frame_id = robot_namespace_frames +

//   static_broadcaster.sendTransform(static_transformStamped);
//   ros::Time t = ros::Time::now();

//   Eigen::Affine3d T = Eigen::Affine3d::Identity();

//   EXPECT_TRUE(l_->getTransformEigenFromTf(
//       robot_namespace_frames + "/map", robot_namespace_frames + "/base_link",
//       t, T));
// }

// TEST_F(ArtifactLocalizationFixtureTest, testEigenWithInvalidFrameId) {
//   static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//   geometry_msgs::TransformStamped static_transformStamped;

//   static_transformStamped.header.stamp = ros::Time::now();
//   static_transformStamped.header.frame_id = robot_namespace_frames + "/map";
//   static_transformStamped.child_frame_id = robot_namespace_frames +
//   "/base_link"; static_transformStamped.transform.rotation.w = 1.0f;
//   static_broadcaster.sendTransform(static_transformStamped);
//   ros::Time t = ros::Time::now();

//   Eigen::Affine3d T = Eigen::Affine3d::Identity();
//   std::string frame1("non_existent_frame_1");
//   std::string frame2("non_existent_frame_2");
//   EXPECT_FALSE(l_->getTransformEigenFromTf(frame1, frame2, t, T));
// }

// TEST_F(ArtifactLocalizationFixtureTest, testValidDepthImageExtraction) {
//   cv::Mat depth(424, 240, CV_16UC1, 1000);
//   std::string label("backpack");

//   ros::param::set("~modes/visual/depth_scale", 0.001);
//   ros::param::set("~modes/visual/depth_sample_ratio", 0.1);
//   Eigen::Affine3d T = Eigen::Affine3d::Identity();
//   EXPECT_TRUE(c_->getCamToObs(30, 30, 40, 40, label, depth, T));
// }

// TEST_F(ArtifactLocalizationFixtureTest, testInvalidDepthImageExtractionNan) {
//   cv::Mat depth(424, 240, CV_16UC1,
//   std::numeric_limits<double>::quiet_NaN()); std::string label("backpack");

//   ros::param::set("~modes/visual/depth_scale", 0.001);
//   ros::param::set("~modes/visual/depth_sample_ratio", 0.1);
//   Eigen::Affine3d T = Eigen::Affine3d::Identity();
//   // EXPECT_FALSE(c_->getCamToObs(30, 30, 40, 40, label, depth, T)); // TODO
//   fix
// }

// TEST_F(ArtifactLocalizationFixtureTest, testInvalidDepthImageExtractionZero)
// {
//   cv::Mat depth(424, 240, CV_16UC1, cv::Scalar(0));
//   std::string label("backpack");

//   ros::param::set("~modes/visual/depth_scale", 0.001);
//   ros::param::set("~modes/visual/depth_sample_ratio", 0.1);
//   Eigen::Affine3d T = Eigen::Affine3d::Identity();
//   // EXPECT_FALSE(c_->getCamToObs(30, 30, 40, 40, label, depth, T)); // TODO
//   fix
// }

// TEST_F(ArtifactLocalizationFixtureTest, getFrameName) {
//   std::string frame("base_link");
//   ros::param::set("~/base_link_frame", "base_link_frame");
//   ros::param::set("~/map_frame", "map_frame");
//   EXPECT_EQ(robot_namespace_nodes + "/base_link_frame",
//   l_->getFrameName(frame)); frame = "map"; EXPECT_EQ(robot_namespace_nodes +
//   "/map_frame", l_->getFrameName(frame));
// }

// TEST_F(ArtifactLocalizationFixtureTest, isPositionValidNominal) {
//   Eigen::Vector3d pos;
//   pos << 1.0, -2.0, 3.0;
//   EXPECT_TRUE(a_->isPositionValid(pos));
// }

// TEST_F(ArtifactLocalizationFixtureTest, isPositionValidHuge) {
//   Eigen::Vector3d pos;
//   pos << 1.0, 1e300, 3.0;
//   EXPECT_FALSE(a_->isPositionValid(pos));
// }

// TEST_F(ArtifactLocalizationFixtureTest, isPositionValidNan) {
//   Eigen::Vector3d pos;
//   pos << 1.0, std::nan("0"), 3.0;
//   EXPECT_FALSE(a_->isPositionValid(pos));
// }

// TEST_F(ArtifactLocalizationFixtureTest, getCameraNameFromFrame) {
//   std::string frame(robot_namespace_frames +
//                     "/camera_front/camera_color_optical_frame");
//   EXPECT_EQ(c_->getCameraNameFromFrame(frame), "camera_front");
// }

// TEST_F(ArtifactLocalizationFixtureTest, isPixelInBoundsTrue) {
//   float pixel[2];
//   pixel[0] = width / 2;
//   pixel[1] = height / 2;
//   EXPECT_TRUE(c_->isPixelInBounds(pixel));
// }

// TEST_F(ArtifactLocalizationFixtureTest, isPixelInBoundsFalse) {
//   float pixel[2];
//   pixel[0] = -1;
//   pixel[1] = height / 2;
//   EXPECT_FALSE(c_->isPixelInBounds(pixel));
//   pixel[0] = width / 2;
//   pixel[1] = -1;
//   EXPECT_FALSE(c_->isPixelInBounds(pixel));
//   pixel[0] = width + 1;
//   pixel[1] = height / 2;
//   EXPECT_FALSE(c_->isPixelInBounds(pixel));
//   pixel[0] = width / 2;
//   pixel[1] = height + 1;
//   EXPECT_FALSE(c_->isPixelInBounds(pixel));
// }

// TEST_F(ArtifactLocalizationFixtureTest, velodynePointCloudToCvMatInFront) {
//   typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//   PointCloud::Ptr pc(new PointCloud);
//   pc->points.push_back(
//       pcl::PointXYZ(0.7071, -0.7071, 0)); // 1m in front of robot
//   sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
//   pcl::toROSMsg(*pc, *msg);
//   msg->header.frame_id = robot_namespace_frames + "/velodyne";
//   cv::Mat im;

//   static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//   geometry_msgs::TransformStamped tf_vel_opt;

//   tf_vel_opt.header.stamp = ros::Time::now();
//   tf_vel_opt.header.frame_id = robot_namespace_frames + "/velodyne";
//   tf_vel_opt.child_frame_id = robot_namespace_frames +
//   "/boson/camera_optical_frame";
//   // vel has 45 deg CCW rotation about body Z
//   tf_vel_opt.transform.rotation.x = 0.2705981;
//   tf_vel_opt.transform.rotation.y = -0.6532815;
//   tf_vel_opt.transform.rotation.z = 0.6532815;
//   tf_vel_opt.transform.rotation.w = -0.2705981;

//   static_broadcaster.sendTransform(tf_vel_opt);

//   c_->velodynePointCloudToCvMat(msg, im);
//   EXPECT_EQ(cv::countNonZero(im), 1);
// }

// TEST_F(ArtifactLocalizationFixtureTest, velodynePointCloudToCvMatBehind) {
//   ros::param::set("~modes/visual/depth_scale", 0.001);

//   typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//   PointCloud::Ptr pc(new PointCloud);
//   pc->points.push_back(pcl::PointXYZ(-0.7071, 0.7071, 0)); // 1m behind robot
//   sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
//   pcl::toROSMsg(*pc, *msg);
//   msg->header.frame_id = robot_namespace_frames + "/velodyne";
//   cv::Mat im;

//   static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//   geometry_msgs::TransformStamped tf_vel_opt;

//   tf_vel_opt.header.stamp = ros::Time::now();
//   tf_vel_opt.header.frame_id = robot_namespace_frames + "/velodyne";
//   tf_vel_opt.child_frame_id = robot_namespace_frames +
//   "/boson/camera_optical_frame";
//   // vel has 45 deg CCW rotation about body Z
//   tf_vel_opt.transform.rotation.x = 0.2705981;
//   tf_vel_opt.transform.rotation.y = -0.6532815;
//   tf_vel_opt.transform.rotation.z = 0.6532815;
//   tf_vel_opt.transform.rotation.w = -0.2705981;

//   static_broadcaster.sendTransform(tf_vel_opt);

//   c_->velodynePointCloudToCvMat(msg, im);
//   EXPECT_EQ(cv::countNonZero(im), 0);
// }

// TEST_F(ArtifactLocalizationFixtureTest, velodynePointCloudResetImage) {
//   ros::param::set("~modes/visual/depth_scale", 0.001);

//   typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//   PointCloud::Ptr pc(new PointCloud);
//   pc->points.push_back(pcl::PointXYZ(0.7071, -0.7071, 0)); // 1m in front of
//   robot pc->points.push_back(pcl::PointXYZ(0.7071, -0.7071, 0.1)); // 1m in
//   front of robot pc->points.push_back(pcl::PointXYZ(0.7071, -0.7071, -0.1));
//   // 1m in front of robot sensor_msgs::PointCloud2Ptr msg(new
//   sensor_msgs::PointCloud2); pcl::toROSMsg(*pc, *msg); msg->header.frame_id =
//   robot_namespace_frames + "/velodyne"; cv::Mat im;

//   static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//   geometry_msgs::TransformStamped tf_vel_opt;

//   tf_vel_opt.header.stamp = ros::Time::now();
//   tf_vel_opt.header.frame_id = robot_namespace_frames + "/velodyne";
//   tf_vel_opt.child_frame_id = robot_namespace_frames +
//   "/boson/camera_optical_frame";
//   // vel has 45 deg CCW rotation about body Z
//   tf_vel_opt.transform.rotation.x = 0.2705981;
//   tf_vel_opt.transform.rotation.y = -0.6532815;
//   tf_vel_opt.transform.rotation.z = 0.6532815;
//   tf_vel_opt.transform.rotation.w = -0.2705981;

//   static_broadcaster.sendTransform(tf_vel_opt);

//   c_->velodynePointCloudToCvMat(msg, im);
//   EXPECT_EQ(cv::countNonZero(im), 3);

//   // Process the second cloud - only 1 point
//   PointCloud::Ptr pc2(new PointCloud);
//   pc2->points.push_back(pcl::PointXYZ(0.671, -0.6071, 0)); // 1m in front of
//   robot pcl::toROSMsg(*pc2, *msg); msg->header.frame_id =
//   robot_namespace_frames + "/velodyne"; cv::Mat im2;

//   c_->velodynePointCloudToCvMat(msg, im2);
//   EXPECT_EQ(cv::countNonZero(im2), 1);

// }

// TEST_F(ArtifactLocalizationFixtureTest, generateUniqueIds) {
//   std::string frame(robot_namespace_frames +
//                     "/camera_front/camera_color_optical_frame");
//   EXPECT_EQ(c_->getCameraNameFromFrame(frame), "camera_front");
// }

// TEST_F(ArtifactLocalizationFixtureTest, tallyByLabel) {
//   l_->updateTallyByLabel("Backpack");
//   // one that is seen for the first time
//   EXPECT_EQ(a_->getTallyByLabel("Backpack"), "001");
//   l_->updateTallyByLabel("Backpack");
//   // one that has had a repeat observation
//   EXPECT_EQ(a_->getTallyByLabel("Backpack"), "002");
//   // one that hasn't been seen yet
//   EXPECT_EQ(a_->getTallyByLabel("Drill"), "000");

//   // composite
//   EXPECT_EQ(a_->generateUniqueId("Backpack"), "bp002_h1_");
//   EXPECT_EQ(a_->generateUniqueId("Drill"), "dr000_h1_");

//   // other categories
//   l_->updateTallyByLabel("Survivor");
//   EXPECT_EQ(a_->generateUniqueId("Survivor"), "su001_h1_");
//   l_->updateTallyByLabel("Gas");
//   EXPECT_EQ(a_->generateUniqueId("Gas"), "gs001_h1_");
//   l_->updateTallyByLabel("Vent");
//   EXPECT_EQ(a_->generateUniqueId("Vent"), "ve001_h1_");
//   l_->updateTallyByLabel("Cell Phone");
//   EXPECT_EQ(a_->generateUniqueId("Cell Phone"), "cp001_h1_");
// }

// TEST_F(ArtifactLocalizationFixtureTest, shortenArtifactLabel) {
//   EXPECT_EQ(a_->shortenArtifactLabel("Backpack"), "bp");
//   EXPECT_EQ(a_->shortenArtifactLabel("Fire Extinguisher"), "fe");
//   EXPECT_EQ(a_->shortenArtifactLabel("Survivor"), "su");
//   EXPECT_EQ(a_->shortenArtifactLabel("Drill"), "dr");
//   EXPECT_EQ(a_->shortenArtifactLabel("Gas"), "gs");
//   EXPECT_EQ(a_->shortenArtifactLabel("Vent"), "ve");
//   EXPECT_EQ(a_->shortenArtifactLabel("Cell Phone"), "cp");
//   EXPECT_EQ(a_->shortenArtifactLabel("Nonsense Artifact"), "na");
// }

// TEST_F(ArtifactLocalizationFixtureTest, getDepth) {
//   EXPECT_FLOAT_EQ(c_->getDepth(0, 10, 10), 0.0); // test for 0 denominator
//   EXPECT_FLOAT_EQ(c_->getDepth(10, -1, -1), 0.0); // test for nonsense
//   lengths
// }

// TEST_F(ArtifactLocalizationFixtureTest, getArea) {
//   cv::Mat im;
//   int area;
//   EXPECT_FALSE(c_->getArea(im, 0, area)); // test for empty cv::Mat
//   // test for more than 1 channel
//   im = cv::Mat(height, width, CV_8UC3, cv::Scalar(100));
//   EXPECT_FALSE(c_->getArea(im, 0, area));
//   im = cv::Mat(height, width, CV_8UC1, cv::Scalar(100));
//   uchar inten_thresh = 50;
//   EXPECT_TRUE(c_->getArea(im, inten_thresh, area));
// }

// /*
// TEST_F(ArtifactLocalizationFixtureTest, getAveGrayThresh) {
//   cv::Mat im;
//   EXPECT_EQ(c_->getAveGrayThresh(im), 0); // test for empty cv::Mat
//   uchar val = 100;
//   im = cv::Mat(height, width, CV_8UC1, cv::Scalar(val));
//   EXPECT_EQ(c_->getAveGrayThresh(im), val);
// }
// */

// TEST_F(ArtifactLocalizationFixtureTest, getHullArea) {
//   cv::Mat im;
//   uchar thresh = 50;
//   int area;
//   EXPECT_FALSE(c_->getHullArea(im, thresh, area)); // test for empty cv::Mat
//   im = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
//   EXPECT_FALSE(c_->getHullArea(im, thresh, area)); // test for all zeros
//   int xmin, ymin, xmax, ymax;
//   uchar val = 75;
//   xmin = 10;
//   ymin = 10;
//   xmax = width/2;
//   ymax = height/2;
//   for (int i = ymin; i < ymax; i++){
//     for (int j = xmin; j < xmax; j++){
//       im.at<uchar>(i, j) = val;
//     }
//   }
//   EXPECT_TRUE(c_->getHullArea(im, thresh, area));
// }

// TEST_F(ArtifactLocalizationFixtureTest, getLen) {
//   cv::Mat im;
//   int xmin, ymin, xmax, ymax;
//   float len;
//   xmin = 10;
//   ymin = 10;
//   xmax = width/2;
//   ymax = height/2;
//   EXPECT_FALSE(c_->getLen(im, xmin, xmax, ymin, ymax, len)); // test for
//   empty cv::Mat im = cv::Mat(height, width, CV_8UC3, cv::Scalar(0));
//   EXPECT_FALSE(c_->getLen(im, xmin, xmax, ymin, ymax, len)); // test for
//   cv::Mat with > 1 channel im = cv::Mat(height, width, CV_8UC1,
//   cv::Scalar(0)); uchar val = 100; for (int i = ymin; i < ymax; i++){
//     for (int j = xmin; j < xmax; j++){
//       im.at<uchar>(i, j) = val;
//     }
//   }
//   EXPECT_TRUE(c_->getLen(im, xmin, xmax, ymin, ymax, len));
// }

// TEST_F(ArtifactLocalizationFixtureTest, getLen) {
//   cv::Mat im;
//   int xmin, ymin, xmax, ymax;
//   float len;
//   xmin = 10;
//   ymin = 10;
//   xmax = width/2;
//   ymax = height/2;
//   EXPECT_FALSE(c_->getLen(im, xmin, xmax, ymin, ymax, len)); // test for
//   empty cv::Mat im = cv::Mat(height, width, CV_8UC3, cv::Scalar(0));
//   EXPECT_FALSE(c_->getLen(im, xmin, xmax, ymin, ymax, len)); // test for
//   cv::Mat with > 1 channel im = cv::Mat(height, width, CV_8UC1,
//   cv::Scalar(0)); uchar val = 100; for (int i = ymin; i < ymax; i++){
//     for (int j = xmin; j < xmax; j++){
//       im.at<uchar>(i, j) = val;
//     }
//   }
//   EXPECT_TRUE(c_->getLen(im, xmin, xmax, ymin, ymax, len));
// }
