^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cartographer_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-06-01)
----------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.3.0...1.0.0

Forthcoming
-----------
* args for asserts writer backpack 3d
* assets writer launch conversion
* fix assets_writer for MultiEchoLaserScan and PointCloud2
* all bags launch files
* fix demo_2d.rviz and set_remap for offline demo
* offline_backpack_2d.launch conversion
* proper cmakelists install
* fix offline node remappings
* removed unused function arg + comment
* fix deserialization
* got rid of most stderr warnings
* de-boostify
* RCL_ROS_TIME
* clean comments not useful
* carto_ros as proper lib
* revert change in pbstream map node
* fixes to offline node and pbstream to map
* more ros1 dep fix
* some tries
* remove debug compil options
* ros2 conversion
* Check if we already publihed transformation at same timestamp Fix `#1555 <https://github.com/ubica-robotics/cartographer_ros/issues/1555>`_ (`#1556 <https://github.com/ubica-robotics/cartographer_ros/issues/1556>`_)
  Co-authored-by: stribor14 <mirko.kokot@romb-technologies.hr>
* Use the MapBuilder factory function. (`#1551 <https://github.com/ubica-robotics/cartographer_ros/issues/1551>`_)
  This follows `cartographer-project/cartographer#1776 <https://github.com/cartographer-project/cartographer/issues/1776>`_.
* Use PoseGraphInterface instead of PoseGraph. (`#1547 <https://github.com/ubica-robotics/cartographer_ros/issues/1547>`_)
  Tiny improvement to not depend on more than is needed.
* Make publishing tf optional, enable publishing PoseStamped (`#1099 <https://github.com/ubica-robotics/cartographer_ros/issues/1099>`_)
  If the output of cartographer should be used as an input to an additional sensor fusion method,
  using the published TF transform is inconvenient or in our specific use case even harmful
  because we don't want to add the raw cartographer output to our TF tree.
  With this change it becomes optional to broadcast to /tf.
  Morever there is an option to publish the tracked frame pose as a PoseStamped message.
  We add two new optional parameters:
  - `publish_to_tf` if false no tf transform is broadcasted
  -  `publish_tracked_pose` If set `true` a PoseStamped representing the position of the
  tracked pose w.r.t. map frame is published.
  If default launchers and settings are used this PR causes no change in the behavior.
* Follow `cartographer-project/cartographer#1759 <https://github.com/cartographer-project/cartographer/issues/1759>`_. (`#1525 <https://github.com/ubica-robotics/cartographer_ros/issues/1525>`_)
* Fix includes to follow style guide in playable_bag.h (`#1524 <https://github.com/ubica-robotics/cartographer_ros/issues/1524>`_)
* Follow `cartographer-project/cartographer#1749 <https://github.com/cartographer-project/cartographer/issues/1749>`_. (`#1516 <https://github.com/ubica-robotics/cartographer_ros/issues/1516>`_)
* Changes for ROS Noetic support (`#1494 <https://github.com/ubica-robotics/cartographer_ros/issues/1494>`_)
* Prepare GMock support for Noetic. (`#1499 <https://github.com/ubica-robotics/cartographer_ros/issues/1499>`_)
  This follows Cartographer installing libgmock-dev.
* Build Abseil dependency in CI. (`#1485 <https://github.com/ubica-robotics/cartographer_ros/issues/1485>`_)
  `cartographer-project/cartographer#1711 <https://github.com/cartographer-project/cartographer/issues/1711>`_ removed
  building Abseil from the Cartographer library.
  We follow here the same approach as in
  cartographer CI.
  An alternative approach which is now possible is
  adding an Abseil package for catkin and depending
  on that.
* Fix build status on front page. (`#1483 <https://github.com/ubica-robotics/cartographer_ros/issues/1483>`_)
  * Fix build status on front page.
  Changes "googlecartographer" to "cartographer-project"
  for references to CI and GitHub.
  Following `cartographer-project/cartographer#1693 <https://github.com/cartographer-project/cartographer/issues/1693>`_.
  * Remove Indigo and Lunar Dockerfiles.
* Remove unnecessary eigen_conversions dependency. (`#1278 <https://github.com/ubica-robotics/cartographer_ros/issues/1278>`_)
  Seems to have been used in the past, but isn't needed anymore.
* Simplify start_trajectory service (RFC-28) (`#1245 <https://github.com/ubica-robotics/cartographer_ros/issues/1245>`_)
  * Simplify start_trajectory.
  * Ran clang-format.
  * Make corrections based on the review
  * Make corrections based on review `#2 <https://github.com/ubica-robotics/cartographer_ros/issues/2>`_, remove obsolete functions
  * Remove unnecessary local variables
* Service to query trajectory segments from the pose graph. (`#1222 <https://github.com/ubica-robotics/cartographer_ros/issues/1222>`_)
  This `/trajectory_query` service allows to look up trajectory segments
  from the pose graph.
  This can be useful if one wants to get optimized trajectory data from
  the current SLAM run and listening to live TF data would be too noisy.
  For example, to stitch buffered point cloud data from a depth sensor
  based on a recent localization trajectory segment of a robot.
  Before, the pose graph trajectory nodes were not available except for
  the trajectory marker topic (only positions, no orientation; inefficient)
  or after serialization (which is not practical in live operation).
* Add explicit --save_state_filename to offline node options. (`#1204 <https://github.com/ubica-robotics/cartographer_ros/issues/1204>`_)
  This allows to serialize the state also when no bagfile is given, e.g.
  when the offline node is used to run an optimization and/or trimming
  configuration on a pbstream. Or simply when one wants to use a custom
  name directly. This doesn't break compatibility with the previous CLI.
* Unify trajectory state checks for service handlers. (`#1262 <https://github.com/ubica-robotics/cartographer_ros/issues/1262>`_)
  Some service handlers need to check if a trajectory is in a valid
  state for the requested operation. If it's not, they return an error
  response.
  `Node::CheckTrajectoryState` allows to avoid code duplication in such
  situations.
* Update clang-format to new Google style. (`#1260 <https://github.com/ubica-robotics/cartographer_ros/issues/1260>`_)
  Apparently the format bot uses a bleeding edge clang-format that uses
  the new Google style and reformats a bunch of files in every PR. This is
  an empty commit to trigger this in a separate commit.
  See https://github.com/llvm-mirror/clang/commit/62e3198c4f5490a1c60ba51d81fe2e1f0dc99135
* Extend trajectory export tool to write TransformStamped topics. (`#1169 <https://github.com/ubica-robotics/cartographer_ros/issues/1169>`_)
  See discussion in `#1166 <https://github.com/ubica-robotics/cartographer_ros/issues/1166>`_.
* Replace a few string operator+ by absl::StrCat(). (`#1244 <https://github.com/ubica-robotics/cartographer_ros/issues/1244>`_)
  ...in some places that can be called frequently.
* Fix segfault by changing the destruction order. (`#1235 <https://github.com/ubica-robotics/cartographer_ros/issues/1235>`_)
  The metrics registry is used as a raw pointer reference in map builder
  and must outlive it.
  Since C++ destroys in reverse declaration order, we have to put the
  registry declaration before the map builder bridge.
  Fixes `#1234 <https://github.com/ubica-robotics/cartographer_ros/issues/1234>`_.
* Tool for extracting pbstream trajectories into bag with tf. (`#1166 <https://github.com/ubica-robotics/cartographer_ros/issues/1166>`_)
  For every trajectory, writes tf
  FLAGS_parent_frame --> trajectory\_`trajectory_id`
* Publish one last progress message when PlayableBag is finished. (`#1160 <https://github.com/ubica-robotics/cartographer_ros/issues/1160>`_)
  Without this, it might look like the processing hangs.
* Don't run final optimization in visualize_pbstream.launch (`#1157 <https://github.com/ubica-robotics/cartographer_ros/issues/1157>`_)
  * Don't run final optimization in visualize_pbstream.launch
  Replaces the offline node with the normal node.
  The problem is that the offline node immediately runs a final
  optimization with `visualize_pbstream.lua`, which is most likely not the
  configuration that was used to generate the pbstream. This can lead to
  effects like distortions in the map e.g. due to different weights, even
  though the actual saved state is fine.
  * Drop all /echoes or /imu messages.
  * Use -start_trajectory_with_default_topics=false
* Windows build & Azure CI (`#1103 <https://github.com/ubica-robotics/cartographer_ros/issues/1103>`_)
* Configurable frame IDs in trajectory_comparison_main.cc (`#1120 <https://github.com/ubica-robotics/cartographer_ros/issues/1120>`_)
  Fixes also the other flag descriptions.
* Add git dependency to package.xml (for Abseil build). (`#1098 <https://github.com/ubica-robotics/cartographer_ros/issues/1098>`_)
* Consider waiting trajectories with a sensor bridge as active. (`#1089 <https://github.com/ubica-robotics/cartographer_ros/issues/1089>`_)
  * Consider waiting trajectories with a sensor bridge as active.
  Fixes a corner case where trajectories that didn't start SLAMing yet
  couldn't be finished, e.g. due to waiting for sensor data. They don't
  appear in the trajectory states list of the pose graph yet but already
  have a trajectory builder.
  https://github.com/googlecartographer/cartographer/issues/1367
* Only include correct source files in cmake (`#1085 <https://github.com/ubica-robotics/cartographer_ros/issues/1085>`_)
  This fixes `#1050 <https://github.com/ubica-robotics/cartographer_ros/issues/1050>`_. Tested standalone compilation and with the debian fakeroot tool. I had to build with a custom Protobuf3 instance though, so another build-check on a regular setup would be appreciated.
  Before this change all *.cc files would be included. If subprojects
  were run individually with the Debian package generator. This resulted in an
  inclusion of temporarily checked out *.cc from the abseil include. This
  change fixes the import behaviour and enables the creation of a valid
  package.
* Add a script for testing with fake landmarks to scripts/dev (`#1071 <https://github.com/ubica-robotics/cartographer_ros/issues/1071>`_)
  See `./publish_fake_random_landmarks.py --help` for documentation.
* Transform landmark poses to the tracking frame. (`#1076 <https://github.com/ubica-robotics/cartographer_ros/issues/1076>`_)
  * Transform landmark poses to the tracking frame.
  * Address the comment.
* Remove orphaned function in occupancy_grid_node_main.cc (`#1034 <https://github.com/ubica-robotics/cartographer_ros/issues/1034>`_)
  Follow-up to `#715 <https://github.com/ubica-robotics/cartographer_ros/issues/715>`_, spotted by @CccYunxiao, see `#1031 <https://github.com/ubica-robotics/cartographer_ros/issues/1031>`_.
* Decrease asset writer progress log period (`#1044 <https://github.com/ubica-robotics/cartographer_ros/issues/1044>`_)
  With two 20 Hz lidars and a 200 Hz IMU, the progress reports were outputted every 500 seconds, which is really too sparse to be useful. Decrease this by a factor of 10.
* Fix Clang thread-safety warning. (`#1068 <https://github.com/ubica-robotics/cartographer_ros/issues/1068>`_)
  reading variable 'submap_slices\_' requires holding mutex 'mutex\_' [-Wthread-safety-analysis]
* Adding launch file arg for launch-prefix to offline nodes (`#1066 <https://github.com/ubica-robotics/cartographer_ros/issues/1066>`_)
  Useful for debugging with gdb or profiling, e.g. with perf.
* Only use ROS log sink in pbstream_map_publisher_main.cc (`#1040 <https://github.com/ubica-robotics/cartographer_ros/issues/1040>`_)
  Fixes double logging to stderr and rosout and makes it consistent with other
  cartographer_ros nodes.
* fix: Use an explicit message_counter instead of using `std::distance` (`#1045 <https://github.com/ubica-robotics/cartographer_ros/issues/1045>`_)
  As @ojura reported and explained in `#940 <https://github.com/ubica-robotics/cartographer_ros/issues/940>`_, there is a critical issue with the new changes introduced in the mentioned PR, which significantly slows down the offline node.
  - The reason of this problem was that In order to count the number of processed messages the `std::distance` function was used which is computationally expensive(O(n)).
  - Instead, the former `log_counter\_` and now `message_counter\_` class variable which was used to print a message every X seconds is also employed to count the number of the processed (and also skipped) messages.
* Use absl::SkipEmpty() predicate. (`#1042 <https://github.com/ubica-robotics/cartographer_ros/issues/1042>`_)
  Fixes empty splits for default "" arguments. Follow up to `#1026 <https://github.com/ubica-robotics/cartographer_ros/issues/1026>`_, thx to @ojura.
* Replace custom SplitString() by absl::StrSplit() (`#1026 <https://github.com/ubica-robotics/cartographer_ros/issues/1026>`_)
  code simplification
* feat: Publish progress of processing the bagfile (`#940 <https://github.com/ubica-robotics/cartographer_ros/issues/940>`_)
* Follow `googlecartographer/cartographer#1424 <https://github.com/googlecartographer/cartographer/issues/1424>`_ (`#1014 <https://github.com/ubica-robotics/cartographer_ros/issues/1014>`_)
* Disable forwarding proto stream by default in node_grpc_main.cc (`#1013 <https://github.com/ubica-robotics/cartographer_ros/issues/1013>`_)
  ...and use `LoadStateFromFile` from `MapBuilderStub`. The `LoadState` stub
  that's used in `Node::LoadState` streams the state file instead, which can now
  be used by explicitly setting the `--upload_load_state_file` flag.
  Solves problems with the gRPC message size limit when loading large state files.
* Add --load_frozen_state to node_grpc_main.cc (`#973 <https://github.com/ubica-robotics/cartographer_ros/issues/973>`_)
* Removing unless from argument (`#994 <https://github.com/ubica-robotics/cartographer_ros/issues/994>`_)
  The roslaunch API throws an exception if setting the argument but not using it.
  This now forces any including launch files to provide an argument for it, even if it might not be used in the `no_rviz` case.
* Adding generic parametrizable offline_node.launch (`#983 <https://github.com/ubica-robotics/cartographer_ros/issues/983>`_)
  And adapting offline_backpack\_(2d|3d).launch to use it.
  This will be useful for parametrizing evaluation runs as well.
* Follow cartographer`#1357 <https://github.com/ubica-robotics/cartographer_ros/issues/1357>`_ (`#964 <https://github.com/ubica-robotics/cartographer_ros/issues/964>`_)
* Adding option to launch without rviz, similar to 2d case (`#972 <https://github.com/ubica-robotics/cartographer_ros/issues/972>`_)
* [ABSL] Use absl::Mutex. (`#969 <https://github.com/ubica-robotics/cartographer_ros/issues/969>`_)
* Add include_unfinished_submaps parameter to SerializeState() (`#966 <https://github.com/ubica-robotics/cartographer_ros/issues/966>`_)
  - default to false in gRPC node (unsupported in `MapBuilderStub`)
  - default to true in classic ROS nodes (as it was before)
  - add as parameter to `write_state`
* Follow `googlecartographer/cartographer#1353 <https://github.com/googlecartographer/cartographer/issues/1353>`_ (`#959 <https://github.com/ubica-robotics/cartographer_ros/issues/959>`_)
  FIXES=`#944 <https://github.com/ubica-robotics/cartographer_ros/issues/944>`_
* Follow `googlecartographer/cartographer#1352 <https://github.com/googlecartographer/cartographer/issues/1352>`_ (`#957 <https://github.com/ubica-robotics/cartographer_ros/issues/957>`_)
* Follow the Absl update. (`#955 <https://github.com/ubica-robotics/cartographer_ros/issues/955>`_)
* Fix pbstream exporting binaries (`#945 <https://github.com/ubica-robotics/cartographer_ros/issues/945>`_)
  `googlecartographer/cartographer#1286 <https://github.com/googlecartographer/cartographer/issues/1286>`_ modified Submap::ToProto such that grids for unfinished submaps are no longer serialized. This commit fixes the breakage this introduced in the pbstream exporting binaries.
* Add option to disable PoseExtrapolator (`#946 <https://github.com/ubica-robotics/cartographer_ros/issues/946>`_)
  This is useful for tuning/debugging to rule out (simulated) time issues
  (because published pose will then only depend on header times).
  Another use case is when Cartographer runs on a separate machine
  that has a different system clock than the sensors.
* Get rid of std::bind. (`#939 <https://github.com/ubica-robotics/cartographer_ros/issues/939>`_)
  `std::bind` is bug prone and should be avoided.
  Lambdas are a more general and safer replacement.
  Similar to `googlecartographer/cartographer#1261 <https://github.com/googlecartographer/cartographer/issues/1261>`_.
* Move conversion table to LoadOccupancyGridMap. (`#941 <https://github.com/ubica-robotics/cartographer_ros/issues/941>`_)
* Add .clang-format file. (`#938 <https://github.com/ubica-robotics/cartographer_ros/issues/938>`_)
  This adds a .clang-format file, so that git clang-format uses
  Google style without the need to remember the commandline flag.
  Similar to `googlecartographer/cartographer#1249 <https://github.com/googlecartographer/cartographer/issues/1249>`_.
* Introduce value converter tables. (`#937 <https://github.com/ubica-robotics/cartographer_ros/issues/937>`_)
* Warn for possible topic mismatch (`#935 <https://github.com/ubica-robotics/cartographer_ros/issues/935>`_)
  FIXES=`#929 <https://github.com/ubica-robotics/cartographer_ros/issues/929>`_
* Simplify gauge and histogram implementation. (`#922 <https://github.com/ubica-robotics/cartographer_ros/issues/922>`_)
  Use mutex locker instead of atomic operations in Gauge.
  Remove unnecessary constructor overload from Histogram.
* remove unused declaration (`#934 <https://github.com/ubica-robotics/cartographer_ros/issues/934>`_)
* Follow `googlecartographer/cartographer#1241 <https://github.com/googlecartographer/cartographer/issues/1241>`_ (`#923 <https://github.com/ubica-robotics/cartographer_ros/issues/923>`_)
* Allow zero pose_publish_period (`#933 <https://github.com/ubica-robotics/cartographer_ros/issues/933>`_)
  To compare different SLAM software online, it is necessary to
  disable tf broadcast.
  Because we already have a parameter "pose_publish_period_sec",
  we use a zero value here to turn off tf broadcast.
* Use 'landmarks' instead of 'landmark'. (`#931 <https://github.com/ubica-robotics/cartographer_ros/issues/931>`_)
* Fix bug in FinishTrajectory logic (`#926 <https://github.com/ubica-robotics/cartographer_ros/issues/926>`_)
  This PR adds additional bookkeeping for trajectories that we scheduled for
  finishing.
  In Node::RunFinalOptimization(...), we were calling FinishTrajectory for
  every active trajectory (state == ACTIVE). Since the state only gets updated
  once the corresponding worker for the FinishTrajectory task is
  scheduled, we were potentially calling FinishTrajectory twice for a
  single trajectory id.
  Reproducible on master e.g. with
  ```
  roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=b2-2016-02-02-14-01-56.bag no_rviz:=true
  ```
* Update msg_conversion.cc (`#925 <https://github.com/ubica-robotics/cartographer_ros/issues/925>`_)
* Register internal metrics and provide a public interface. (`#917 <https://github.com/ubica-robotics/cartographer_ros/issues/917>`_)
  [RFC 24](https://github.com/googlecartographer/rfcs/blob/master/text/0024-monitoring-ros.md)
  Public API:
  - adds `cartographer_ros::metrics::FamilyFactory`
  - compatible with `::cartographer::metrics::RegisterAllMetrics`
  Public RPC interface:
  - adds the ROS service `/read_metrics`
  - response contains the latest values of all available metric families
* Use new pure localization trimmer options. (`#918 <https://github.com/ubica-robotics/cartographer_ros/issues/918>`_)
* Add internal metric families. (`#914 <https://github.com/ubica-robotics/cartographer_ros/issues/914>`_)
  - minimal counter, gauge and histogram implementations
  - metric family interfaces as defined in libcartographer
  - serializable to ROS messages
  RFC: https://github.com/googlecartographer/rfcs/pull/26
* Allow to ignore (un-)frozen submaps in the occupancy grid node. (`#899 <https://github.com/ubica-robotics/cartographer_ros/issues/899>`_)
* Discard proto data in pbstream_map_publisher via RAII. (`#912 <https://github.com/ubica-robotics/cartographer_ros/issues/912>`_)
  We don't need it after the occupancy grid is drawn.
  Reduces the memory consumption especially for large maps.
* Use PoseGraphInterface::TrajectoryState from libcartographer (`#910 <https://github.com/ubica-robotics/cartographer_ros/issues/910>`_)
  https://github.com/googlecartographer/rfcs/pull/35
  - makes use of the trajectory state in `map_builder` and `node`
  - adds a service to query the trajectory states
  - follow-up to https://github.com/googlecartographer/cartographer/pull/1214
  that takes the deleted state into account in the `/finish_trajectory` service
  (could crash otherwise)
* Improve internal naming of local SLAM data. (`#908 <https://github.com/ubica-robotics/cartographer_ros/issues/908>`_)
  Prevents confusion with TrajectoryState and GetTrajectoryStates()
  of the pose graph interface. The affected data is only local.
* Revert timers other than PublishTrajectoryStates back to being WallTimers. (`#898 <https://github.com/ubica-robotics/cartographer_ros/issues/898>`_)
* Ensure we validate what we CHECK(...) (`#897 <https://github.com/ubica-robotics/cartographer_ros/issues/897>`_)
  In cartographer we check for strict ordering, i.e. do not allow
  subsequent timestamps to be exactly equal. This fixes the rosbag validation tool
  to do the same.
* Use timing channel from PointCloud2, if available.  (`#896 <https://github.com/ubica-robotics/cartographer_ros/issues/896>`_)
* Fix memory leak in simulations by removing wall timers. (`#891 <https://github.com/ubica-robotics/cartographer_ros/issues/891>`_)
  Fixes the problem of ever-growing memory after `rosbag play --clock` finishes,
  as discussed in https://github.com/googlecartographer/cartographer/issues/1182
  The wall timers caused the timer callback that publishes TF data to be called
  even if no simulated `/clock` was published anymore.
  As the TF buffer cache time of the TF listener seems to be based on
  the ROS time instead of wall clock, it could grow out of bounds.
  Now, `ros::Timer` plays nicely with both normal (wall) and simulated time and
  no callbacks are executed if `/clock` stops in simulation.
* set required version of dependencies (`#892 <https://github.com/ubica-robotics/cartographer_ros/issues/892>`_)
* remove architecture specific definitions exported by PCL (`#893 <https://github.com/ubica-robotics/cartographer_ros/issues/893>`_)
  * remove architecture specific definitions exported by PCL
  This is an issue on PCL 1.8.X causing SIGILL, Illegal instruction crashes: https://github.com/ros-gbp/cartographer_ros-release/issues/10
  Should be fixed in future PCL version with https://github.com/PointCloudLibrary/pcl/pull/2100
* Release 1.0. (`#889 <https://github.com/ubica-robotics/cartographer_ros/issues/889>`_)
* Add maintainers and authors to package.xml (`#886 <https://github.com/ubica-robotics/cartographer_ros/issues/886>`_)
* Follow `googlecartographer/cartographer#1174 <https://github.com/googlecartographer/cartographer/issues/1174>`_ (`#883 <https://github.com/ubica-robotics/cartographer_ros/issues/883>`_)
  Update all users to the new serialization format [RFC 0021](https://github.com/googlecartographer/rfcs/blob/master/text/0021-serialization-format.md)
  See also corresponding change in cartographer: `googlecartographer/cartographer#1174 <https://github.com/googlecartographer/cartographer/issues/1174>`_
* Follow `googlecartographer/cartographer#1172 <https://github.com/googlecartographer/cartographer/issues/1172>`_ (`#881 <https://github.com/ubica-robotics/cartographer_ros/issues/881>`_)
* Sanitize node memory consumption with a smaller TF buffer size. (`#879 <https://github.com/ubica-robotics/cartographer_ros/issues/879>`_)
  Fixes an (almost) unbounded growth of the TF buffer.
  See the heap profile logs in the PR for more information.
* Follow `googlecartographer/cartographer#1164 <https://github.com/googlecartographer/cartographer/issues/1164>`_ (`#877 <https://github.com/ubica-robotics/cartographer_ros/issues/877>`_)
* Assets writer (ROS map) urdf typo fix (`#875 <https://github.com/ubica-robotics/cartographer_ros/issues/875>`_)
  The ROS map assets writer launch file can now find the default urdf file when no argument is provided.
* Fix the 'load_frozen_state' flag in visualize_pbstream.launch. (`#863 <https://github.com/ubica-robotics/cartographer_ros/issues/863>`_)
* Follow `googlecartographer/cartographer#1143 <https://github.com/googlecartographer/cartographer/issues/1143>`_ (`#859 <https://github.com/ubica-robotics/cartographer_ros/issues/859>`_)
* Adapt to new mapping proto location of cartographer (`#860 <https://github.com/ubica-robotics/cartographer_ros/issues/860>`_)
* Use immediately invoked lambda for tracking_to_local. (`#848 <https://github.com/ubica-robotics/cartographer_ros/issues/848>`_)
  Restores const-correctness that we dropped when introducing the
  `publish_frame_projected_to_2d` param without using a ternary operator.
* Add cartographer_dev_rosbag_publisher (`#854 <https://github.com/ubica-robotics/cartographer_ros/issues/854>`_)
  This adds a tool to publish a bag file without publishing a simulated clock, modifying header timestamps.
* Follow up on https://github.com/googlecartographer/cartographer/pull/1108 (`#838 <https://github.com/ubica-robotics/cartographer_ros/issues/838>`_)
* Add a launch and configuration file for writing a ROS map (`#577 <https://github.com/ubica-robotics/cartographer_ros/issues/577>`_) (`#721 <https://github.com/ubica-robotics/cartographer_ros/issues/721>`_)
  ( Trying again, accidentally deleted source branch for previous PR )
  Related issue: `#577 <https://github.com/ubica-robotics/cartographer_ros/issues/577>`_
  The min/max range default to the same as the backback_2d examples; same thing for the URDF file.
  However, the name of both the config file and the launch file are kept generic.
* Internal cleanup. (`#821 <https://github.com/ubica-robotics/cartographer_ros/issues/821>`_)
  Fix lint error.
* Registration of external points processors in AssetsWriter (`#830 <https://github.com/ubica-robotics/cartographer_ros/issues/830>`_)
  Added RegisterPointsProcessor method to AssetsWriter class. This allows to register new points processors to the pipeline builder.
  As the new points processors may write files to the disk, the CreateFileWriterFactory method is exposed.
* Extract assets writer class from static method (`#827 <https://github.com/ubica-robotics/cartographer_ros/issues/827>`_)
  Extracted class Assets_Writer from RunAssetsWriterPipeline.
  The idea is to increase the re-usability and flexibility of the assets_writer: In next PR, the assets_writer will allow registering external points_processers to the points processing pipeline. This requires having a class instead of a static method to allow for different states.
* Enable rendering of submaps without a grid (`#829 <https://github.com/ubica-robotics/cartographer_ros/issues/829>`_)
  - related to https://github.com/googlecartographer/cartographer_ros/issues/819
* Assets writer refactoring (`#814 <https://github.com/ubica-robotics/cartographer_ros/issues/814>`_)
  The assets writing method was split into several calls to sub-routines.
  RunAssetsWriterPipeline now calls sub-routines creating objects from files and then runs the pipeline using the created objects. This should increase readability of the method.
* Correct localization_3d.launch (`#824 <https://github.com/ubica-robotics/cartographer_ros/issues/824>`_)
  Also, be consistent with 2D and with documentation.
  ISSUE=https://github.com/googlecartographer/cartographer/issues/1056
* Internal cleanup. (`#818 <https://github.com/ubica-robotics/cartographer_ros/issues/818>`_)
  Move the self header file after system header.
* Take frozen state into account when finishing trajectories. (`#811 <https://github.com/ubica-robotics/cartographer_ros/issues/811>`_)
  Until now, the error response of an /finish_trajectory request for a
  frozen trajectory was 'Trajectory ... is not created yet.'.
  This is a lie. The new response is more accurate because the trajectory
  __is_\_ created, but it just can't be finished because it's frozen.
* Fix race-condition when attempting to fetch trimmed submaps. (`#812 <https://github.com/ubica-robotics/cartographer_ros/issues/812>`_)
  A simple solution for a slightly more complex scenario:
  - a pure localization trajectory `X` gets finished & trimmed in the main node
  - at the same time, the occupancy_grid_node handles an outdated SubmapList
  message in which a submap ID `id` of trajectory `X` is still present
  - the call to FetchSubmapTextures(`id`, ...) leads to a crash
  With this fix, the trimmed submap IDs are just ignored until the next
  iteration (in which the occupancy grid node removes the trimmed IDs).
* moved run method of assets writer main to separate files (`#807 <https://github.com/ubica-robotics/cartographer_ros/issues/807>`_)
  Moved the run method of the assets_writer_main to the separate assets_writer files.
  Will extract asset_writer class in the future to keep the main file small and allow re-usability and more flexibility of the asset_writer.
* Check service status code in start_trajectory_main.cc (`#808 <https://github.com/ubica-robotics/cartographer_ros/issues/808>`_)
  Small patch to distinguish between communication and
  runtime errors when calling the ROS service (as introduced by RFC 13).
* Check overlapping range data correctly (`#804 <https://github.com/ubica-robotics/cartographer_ros/issues/804>`_)
  FIXES=`#771 <https://github.com/ubica-robotics/cartographer_ros/issues/771>`_
* Fix sequential subdivisions (`#806 <https://github.com/ubica-robotics/cartographer_ros/issues/806>`_)
  FIXES=https://github.com/googlecartographer/cartographer/issues/1026
* Tool for comparing pure localization to offline optimization (`#803 <https://github.com/ubica-robotics/cartographer_ros/issues/803>`_)
  Adds a tool to measure the difference between a trajectory from a pbstream and one given by tf messages in a bag file, and a script to evaluate real-time pure localization poses compared to a globally optimized mapping poses.
* Show constraints in rviz (`#789 <https://github.com/ubica-robotics/cartographer_ros/issues/789>`_)
* Launch script to visualize pbstream in rviz (`#788 <https://github.com/ubica-robotics/cartographer_ros/issues/788>`_)
* Add constraint-dependent trajectory visualization. (`#756 <https://github.com/ubica-robotics/cartographer_ros/issues/756>`_)
* Avoid failed CHECK when running offline node with no bags. (`#777 <https://github.com/ubica-robotics/cartographer_ros/issues/777>`_)
  Bug introduced in `#680 <https://github.com/ubica-robotics/cartographer_ros/issues/680>`_.
* Ignore empty laser scan message. (`#767 <https://github.com/ubica-robotics/cartographer_ros/issues/767>`_)
  FIXES=`#766 <https://github.com/ubica-robotics/cartographer_ros/issues/766>`_
* Minor optimizations of cases with no subscribers (`#755 <https://github.com/ubica-robotics/cartographer_ros/issues/755>`_)
* Add time skip option for offline node (`#680 <https://github.com/ubica-robotics/cartographer_ros/issues/680>`_)
  Introduces a "skip" option which skips first _t\_ seconds.
* Follow https://github.com/googlecartographer/cartographer/pull/958. (`#754 <https://github.com/ubica-robotics/cartographer_ros/issues/754>`_)
* Follow https://github.com/googlecartographer/cartographer/pull/955. (`#751 <https://github.com/ubica-robotics/cartographer_ros/issues/751>`_)
* Pass ROS landmark topic to the cartographer. (`#746 <https://github.com/ubica-robotics/cartographer_ros/issues/746>`_)
  [Landmark RFC](https://github.com/googlecartographer/rfcs/blob/master/text/0011-landmarks.md)
* Follow PR [`#950 <https://github.com/ubica-robotics/cartographer_ros/issues/950>`_](https://github.com/googlecartographer/cartographer/pull/950). (`#750 <https://github.com/ubica-robotics/cartographer_ros/issues/750>`_)
  [Internal hdrs RFC](https://github.com/googlecartographer/rfcs/blob/master/text/0003-internal-headers.md)
* Fix pbstream_map_publisher (follow `#712 <https://github.com/ubica-robotics/cartographer_ros/issues/712>`_) (`#745 <https://github.com/ubica-robotics/cartographer_ros/issues/745>`_)
  Applies the proto deserialization changes that
  were introduced in PR `#712 <https://github.com/ubica-robotics/cartographer_ros/issues/712>`_.
* s/LoadMap/LoadState in node_grpc_main.cc (`#744 <https://github.com/ubica-robotics/cartographer_ros/issues/744>`_)
* Offline multi-trajectory: use topic names without 'bag_n\_' prefix by default (`#707 <https://github.com/ubica-robotics/cartographer_ros/issues/707>`_)
  This fixes offline_backpack\_*.launch for multiple bags.
* Use CreateOccupancyGridMsg() in occupancy_grid_node_main.cc (`#715 <https://github.com/ubica-robotics/cartographer_ros/issues/715>`_)
  Follow-up of PR `#711 <https://github.com/ubica-robotics/cartographer_ros/issues/711>`_.
* Unfrozen trajectories (`#710 <https://github.com/ubica-robotics/cartographer_ros/issues/710>`_)
  Unfrozen trajectories
* Fix the path to mapping\_*d includes. (`#736 <https://github.com/ubica-robotics/cartographer_ros/issues/736>`_)
  [Code structure RFC](https://github.com/googlecartographer/rfcs/blob/master/text/0016-code-structure.md)
* Validate tool checks per-point time stamps. (`#737 <https://github.com/ubica-robotics/cartographer_ros/issues/737>`_)
  Checks for per-point timing issues in a bag file.
  Feature is tracked in `#529 <https://github.com/ubica-robotics/cartographer_ros/issues/529>`_.
* Add option to publish a pure 2D pose. (`#683 <https://github.com/ubica-robotics/cartographer_ros/issues/683>`_)
  If the new `publish_frame_projected_to_2d` option is set to true,
  the published pose will be restricted to a pure 2D pose
  (no roll, pitch, or z-offset).
  This prevents potentially unwanted out-of-plane poses in 2D mode
  that can occur due to the pose extrapolation step (e.g. if the pose
  shall be published as a 'base-footprint'-like frame).
* Follow  `googlecartographer/cartographer#922 <https://github.com/googlecartographer/cartographer/issues/922>`_ (`#734 <https://github.com/ubica-robotics/cartographer_ros/issues/734>`_)
  * Follow  `googlecartographer/cartographer#927 <https://github.com/googlecartographer/cartographer/issues/927>`_
* Avoid auto for Eigen expressiongs. (`#719 <https://github.com/ubica-robotics/cartographer_ros/issues/719>`_)
  While harmless in most cases, auto can delay evaluation
  of expressions in unexpected ways.
  So it is better to avoid auto for Eigen expressions.
  https://eigen.tuxfamily.org/dox/TopicPitfalls.html
* RViz settings for landmarks. (`#717 <https://github.com/ubica-robotics/cartographer_ros/issues/717>`_)
  [RFC=0011](https://github.com/googlecartographer/rfcs/blob/master/text/0011-landmarks.md)
* Publish Landmark markers for RViz. (`#713 <https://github.com/ubica-robotics/cartographer_ros/issues/713>`_)
  [RFC=0011](https://github.com/googlecartographer/rfcs/blob/master/text/0011-landmarks.md)
* Add pbstream_map_publisher_main.cc (`#711 <https://github.com/ubica-robotics/cartographer_ros/issues/711>`_)
  Implements [RFC 06](https://github.com/googlecartographer/rfcs/blob/master/text/0006-serve-ros-map-from-pbstream.md)
* Follow `googlecartographer/cartographer#859 <https://github.com/googlecartographer/cartographer/issues/859>`_ (`#712 <https://github.com/ubica-robotics/cartographer_ros/issues/712>`_)
* Refactor ROS service responses. (`#708 <https://github.com/ubica-robotics/cartographer_ros/issues/708>`_)
  Provide a descriptive StatusResponse msg field consisting of
  an gRPC-like StatusCode and message string to the service caller.
  Implements [RFC 13](https://github.com/googlecartographer/rfcs/blob/master/text/0013-improve-ros-service-responses.md).
* Offline node: better support for sequential bags. (`#694 <https://github.com/ubica-robotics/cartographer_ros/issues/694>`_)
  Allow same topics to be used in different bags (a previously supported use case).
  Remove unused variable `current_bag_sensor_topics`.
  Touch up flag descriptions.
  Fixes `#693 <https://github.com/ubica-robotics/cartographer_ros/issues/693>`_.
  pair=@gaschler
* Follow `googlecartographer/cartographer#839 <https://github.com/googlecartographer/cartographer/issues/839>`_ (`#686 <https://github.com/ubica-robotics/cartographer_ros/issues/686>`_)
  Follow change `googlecartographer/cartographer#839 <https://github.com/googlecartographer/cartographer/issues/839>`_ from string to struct SensorId.
  Compute expected sensor ids for multiple trajectories.
  Remove command argument input for sensor ids.
  Make some methods const.
  Clean up.
* Do not forget to finish trajectory if last message is not from a sensor topic (`#681 <https://github.com/ubica-robotics/cartographer_ros/issues/681>`_)
  Bug introduced in `#636 <https://github.com/ubica-robotics/cartographer_ros/issues/636>`_.
* Fix segfault in rosbag_validate (`#685 <https://github.com/ubica-robotics/cartographer_ros/issues/685>`_)
* Add a launch file for 2d localization demo with gRPC. (`#682 <https://github.com/ubica-robotics/cartographer_ros/issues/682>`_)
* Simultaneous offline multi trajectories (`#636 <https://github.com/ubica-robotics/cartographer_ros/issues/636>`_)
  RFC=[0009](https://github.com/googlecartographer/rfcs/pull/4)
* Constraints visualization: Separate inter constraints between separate trajectories (`#634 <https://github.com/ubica-robotics/cartographer_ros/issues/634>`_)
* Fix gflags include in offline nodes (`#677 <https://github.com/ubica-robotics/cartographer_ros/issues/677>`_)
  FIX=`#676 <https://github.com/ubica-robotics/cartographer_ros/issues/676>`_
* Fix gflags include in offline_node.cc (`#676 <https://github.com/ubica-robotics/cartographer_ros/issues/676>`_)
  FIX=`#676 <https://github.com/ubica-robotics/cartographer_ros/issues/676>`_
* Deduplicate loading options for offline node (`#664 <https://github.com/ubica-robotics/cartographer_ros/issues/664>`_)
  This is preparation for `#636 <https://github.com/ubica-robotics/cartographer_ros/issues/636>`_.
  I noticed that there is duplicated code for loading options for the offline and GRPC offline node because they are needed while constructing the map builder for the non-GRPC offline node (and that step is the only difference between the offline node and the GRPC offline node).
  I got around this by passing a map builder factory to `RunOfflineNode` instead, so we can deduplicate the code for loading options by doing it inside `RunOfflineNode`.
* Adding NavSatFix to trajectory builder. (`#666 <https://github.com/ubica-robotics/cartographer_ros/issues/666>`_)
  GPS message is converted first to ECEF, and then to a local frame. The first GPS message defines the local frame.
  PAIR=wohe
  [RFC=0007](https://github.com/googlecartographer/rfcs/blob/master/text/0007-nav-sat-support.md)
* Transform from ECEF to a local frame where z points up. (`#662 <https://github.com/ubica-robotics/cartographer_ros/issues/662>`_)
  For a given latitude and longitude, return a transformation that takes a point in ECEF coordinates to
  a local frame, where the z axis points up.
  PAIR=wohe
  [RFC=0007](https://github.com/googlecartographer/rfcs/blob/master/text/0007-nav-sat-support.md)
* Wiring for sensor_msgs::NavSatFix (`#659 <https://github.com/ubica-robotics/cartographer_ros/issues/659>`_)
  PAIR=wohe
  [RFC=0007](https://github.com/googlecartographer/rfcs/blob/master/text/0007-nav-sat-support.md)
* Adding conversion from WGS84 to ECEF. (`#660 <https://github.com/ubica-robotics/cartographer_ros/issues/660>`_)
  This converts from latitude, longitude, altitude
  to a cartesian coordinate frame.
  [RFC=0007](https://github.com/googlecartographer/rfcs/blob/master/text/0007-nav-sat-support.md)
* Follow `googlecartographer/cartographer#801 <https://github.com/googlecartographer/cartographer/issues/801>`_ (`#657 <https://github.com/ubica-robotics/cartographer_ros/issues/657>`_)
* Add rviz and simtime to gRPC launch file. (`#658 <https://github.com/ubica-robotics/cartographer_ros/issues/658>`_)
* Fix bug in MapBuilderBridge::GetTrajectoryStates() (`#652 <https://github.com/ubica-robotics/cartographer_ros/issues/652>`_)
* Use GetTrajectoryNodePoses and GetAllSubmapPoses in GetConstraintList (`#651 <https://github.com/ubica-robotics/cartographer_ros/issues/651>`_)
* Make MapBuilderBridge use GetAllTrajectoryNodePoses() (`#649 <https://github.com/ubica-robotics/cartographer_ros/issues/649>`_)
* Make MapBuilderBridge::GetSubmapList() use GetAllSubmapPoses() (`#647 <https://github.com/ubica-robotics/cartographer_ros/issues/647>`_)
* Implement offline gRPC bridge. (`#645 <https://github.com/ubica-robotics/cartographer_ros/issues/645>`_)
* Fix path for gRPC server shell script in CMakeLists.txt (`#644 <https://github.com/ubica-robotics/cartographer_ros/issues/644>`_)
* Refactor offline_node_main.cc to prepare for offline bridge. (`#643 <https://github.com/ubica-robotics/cartographer_ros/issues/643>`_)
  [RFC=0002](https://github.com/googlecartographer/rfcs/blob/master/text/0002-cloud-based-mapping-1.md)
* Follow `googlecartographer/cartographer#782 <https://github.com/googlecartographer/cartographer/issues/782>`_ (`#633 <https://github.com/ubica-robotics/cartographer_ros/issues/633>`_)
  Towards [RFC06](https://github.com/googlecartographer/rfcs/blob/master/text/0006-serve-ros-map-from-pbstream.md).
  Migrates
  * `FillSubmapSlice` from `pbstream_to_rosmap_main.cc`
  * `SubmapTexture` logics from cartographer_ros
* Launch grpc client and server (`#641 <https://github.com/ubica-robotics/cartographer_ros/issues/641>`_)
  Adds a launch file to test the entire grpc bridge.
  Here is an example to run:
  ```
  catkin_make_isolated --use-ninja -DBUILD_GRPC=True
  source devel_isolated/setup.bash
  roslaunch cartographer_ros grpc_demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
  ```
  The bash script is a work-around to run an executable outside of the catkin packages.
* Implement cartographer_grpc_node. (`#632 <https://github.com/ubica-robotics/cartographer_ros/issues/632>`_)
* Add BUILD_GRPC CMake flag and ROS-gRPC binary. (`#631 <https://github.com/ubica-robotics/cartographer_ros/issues/631>`_)
* HandleRangefinder time refers to newest point. (`#612 <https://github.com/ubica-robotics/cartographer_ros/issues/612>`_)
  This is necessary so that sensor::Collator queues range data
  after previous odometry and IMU data, and LocalTrajectoryBuilder
  will be able to unwarp each point.
* Follow `googlecartographer/cartographer#736 <https://github.com/googlecartographer/cartographer/issues/736>`_ (`#620 <https://github.com/ubica-robotics/cartographer_ros/issues/620>`_)
* Detect duplicate range data. (`#619 <https://github.com/ubica-robotics/cartographer_ros/issues/619>`_)
  Checks that range data in a bag file changes between frames, which is one of the common mistakes listed in `#529 <https://github.com/ubica-robotics/cartographer_ros/issues/529>`_.
* Fix 0. constant to 0.0 to comply with YAML standard (`#618 <https://github.com/ubica-robotics/cartographer_ros/issues/618>`_)
* Validate IMU, odometry, timing, frame names. (`#615 <https://github.com/ubica-robotics/cartographer_ros/issues/615>`_)
* Follow googlecartographer/cartographer/pull/724. (`#616 <https://github.com/ubica-robotics/cartographer_ros/issues/616>`_)
* Add initial_pose in start_trajectory_main.cc Fixes `#579 <https://github.com/ubica-robotics/cartographer_ros/issues/579>`_ (`#610 <https://github.com/ubica-robotics/cartographer_ros/issues/610>`_)
  Fixes `#579 <https://github.com/ubica-robotics/cartographer_ros/issues/579>`_
  Related to `googlecartographer/cartographer#606 <https://github.com/googlecartographer/cartographer/issues/606>`_
  @damienrg @cschuet I followed most of the comments in `googlecartographer/cartographer#642 <https://github.com/googlecartographer/cartographer/issues/642>`_ except timestamp. Receiving timestamp sounds weird to me because trajectory should not start in past timestamp or future timestamp.
* Contributors: Alexander Belyaev, Alireza, Christoph Schütte, Guilherme Lawless, Guillaume Doisy, Guillaume dev PC, Jihoon Lee, Jonathan Huber, Juraj Oršulić, Kevin Daun, Martin Schwörer, Matthias Loebach, Michael Grupp, Mikael Arguedas, Roel, Sebastian Klose, Steven Palma, Susanne Pielawa, Wolfgang Hess, gaschler, jie, mgladkova, stribor14

0.3.0 (2017-11-23)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.2.0...0.3.0

0.2.0 (2017-06-19)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.1.0...0.2.0

0.1.0 (2017-05-18)
------------------
* First unstable development release
