^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cartographer_ros_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* ros2 conversion
* Fix build status on front page. (`#1483 <https://github.com/ubica-robotics/cartographer_ros/issues/1483>`_)
  * Fix build status on front page.
  Changes "googlecartographer" to "cartographer-project"
  for references to CI and GitHub.
  Following `cartographer-project/cartographer#1693 <https://github.com/cartographer-project/cartographer/issues/1693>`_.
  * Remove Indigo and Lunar Dockerfiles.
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
* move metrics messages at root of msg folder (`#1020 <https://github.com/ubica-robotics/cartographer_ros/issues/1020>`_)
* feat: Publish progress of processing the bagfile (`#940 <https://github.com/ubica-robotics/cartographer_ros/issues/940>`_)
* Add include_unfinished_submaps parameter to SerializeState() (`#966 <https://github.com/ubica-robotics/cartographer_ros/issues/966>`_)
  - default to false in gRPC node (unsupported in `MapBuilderStub`)
  - default to true in classic ROS nodes (as it was before)
  - add as parameter to `write_state`
* List files explicitly in cartographer_ros_msgs/CMakeLists.txt (`#927 <https://github.com/ubica-robotics/cartographer_ros/issues/927>`_)
  `DIRECTORY` by itself doesn't work as reliable as expected.
* Use 'landmarks' instead of 'landmark'. (`#931 <https://github.com/ubica-robotics/cartographer_ros/issues/931>`_)
* Register internal metrics and provide a public interface. (`#917 <https://github.com/ubica-robotics/cartographer_ros/issues/917>`_)
  [RFC 24](https://github.com/googlecartographer/rfcs/blob/master/text/0024-monitoring-ros.md)
  Public API:
  - adds `cartographer_ros::metrics::FamilyFactory`
  - compatible with `::cartographer::metrics::RegisterAllMetrics`
  Public RPC interface:
  - adds the ROS service `/read_metrics`
  - response contains the latest values of all available metric families
* Add internal metric families. (`#914 <https://github.com/ubica-robotics/cartographer_ros/issues/914>`_)
  - minimal counter, gauge and histogram implementations
  - metric family interfaces as defined in libcartographer
  - serializable to ROS messages
  RFC: https://github.com/googlecartographer/rfcs/pull/26
* Allow to ignore (un-)frozen submaps in the occupancy grid node. (`#899 <https://github.com/ubica-robotics/cartographer_ros/issues/899>`_)
* Use PoseGraphInterface::TrajectoryState from libcartographer (`#910 <https://github.com/ubica-robotics/cartographer_ros/issues/910>`_)
  https://github.com/googlecartographer/rfcs/pull/35
  - makes use of the trajectory state in `map_builder` and `node`
  - adds a service to query the trajectory states
  - follow-up to https://github.com/googlecartographer/cartographer/pull/1214
  that takes the deleted state into account in the `/finish_trajectory` service
  (could crash otherwise)
* Release 1.0. (`#889 <https://github.com/ubica-robotics/cartographer_ros/issues/889>`_)
* Add maintainers and authors to package.xml (`#886 <https://github.com/ubica-robotics/cartographer_ros/issues/886>`_)
* [cartographer_ros_msgs] Message dependency fixup (`#882 <https://github.com/ubica-robotics/cartographer_ros/issues/882>`_)
  * add missing std_msgs dependency
  * use full message identifier to match be consistent with the other messages from this package
* [cartographer_ros_msgs] add run dependency on message_runtime (`#800 <https://github.com/ubica-robotics/cartographer_ros/issues/800>`_)
  As per ROS [tutorials](http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages#Dependencies)
  Edit: For future readers, the linked tutorial was referring to `run_depend` from package format 1 as pointed out by @ojura, it has now been updated to refer to `exec_depend` from format 2
* Pass ROS landmark topic to the cartographer. (`#746 <https://github.com/ubica-robotics/cartographer_ros/issues/746>`_)
  [Landmark RFC](https://github.com/googlecartographer/rfcs/blob/master/text/0011-landmarks.md)
* Add a ROS message for landmark observations. (`#732 <https://github.com/ubica-robotics/cartographer_ros/issues/732>`_)
  [Landmark RFC](https://github.com/googlecartographer/rfcs/blob/master/text/0011-landmarks.md)
* Add option to publish a pure 2D pose. (`#683 <https://github.com/ubica-robotics/cartographer_ros/issues/683>`_)
  If the new `publish_frame_projected_to_2d` option is set to true,
  the published pose will be restricted to a pure 2D pose
  (no roll, pitch, or z-offset).
  This prevents potentially unwanted out-of-plane poses in 2D mode
  that can occur due to the pose extrapolation step (e.g. if the pose
  shall be published as a 'base-footprint'-like frame).
* Refactor ROS service responses. (`#708 <https://github.com/ubica-robotics/cartographer_ros/issues/708>`_)
  Provide a descriptive StatusResponse msg field consisting of
  an gRPC-like StatusCode and message string to the service caller.
  Implements [RFC 13](https://github.com/googlecartographer/rfcs/blob/master/text/0013-improve-ros-service-responses.md).
* Wiring for sensor_msgs::NavSatFix (`#659 <https://github.com/ubica-robotics/cartographer_ros/issues/659>`_)
  PAIR=wohe
  [RFC=0007](https://github.com/googlecartographer/rfcs/blob/master/text/0007-nav-sat-support.md)
* Contributors: Alexander Belyaev, Alireza, Guillaume Doisy, Michael Grupp, Mikael Arguedas, Susanne Pielawa, Wolfgang Hess, mgladkova

0.3.0 (2017-11-23)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.2.0...0.3.0

0.2.0 (2017-06-19)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.1.0...0.2.0

0.1.0 (2017-05-18)
------------------
* First unstable development release
