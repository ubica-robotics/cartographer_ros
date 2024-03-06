^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cartographer_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

102.1.1 (2024-03-06)
--------------------

102.1.0 (2024-03-05)
--------------------

102.0.0 (2024-03-05)
--------------------
* proper cmakelists install
* added 100ms timeout lookuptf rviz
* removed unused function arg + comment
* using latest tf available to avoid warning
* fix sharedptr operator bool misuse
* got rid of most stderr warnings
* sync
* cleaned cmake rviz
* clean comments not useful
* carto_ros as proper lib
* clean xml
* working submps
* Q_EMIT / slot working but material error
* more ros1 dep fix
* ros1 dep
* oof
* plugin can be loaded and used but only the submap markers are visualized
* dirty fix
* solved one issue, got another two
* closer
* backward gbd
* solved one issue, got another one
* some other tries. i think the prob is in the linking to abseilcpp
* some tries
* backward changes temp
* add backward ros
* finally compiling but getting segmentation fault
* fix warning
* not compiling again
* compiling but nothing happens
* almost compiling
* Still not compiling but almost there
* wip
* even more wip not compiling
* more wip
* wip not compiling
* wip not compiling
* ros2 conversion
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
* Replace a few string operator+ by absl::StrCat(). (`#1244 <https://github.com/ubica-robotics/cartographer_ros/issues/1244>`_)
  ...in some places that can be called frequently.
* Windows build & Azure CI (`#1103 <https://github.com/ubica-robotics/cartographer_ros/issues/1103>`_)
* fixing compilation under OSX (`#1126 <https://github.com/ubica-robotics/cartographer_ros/issues/1126>`_)
  Surprising that this has not lead to any issues yet.
  This just works by chance on other systems if QT has been find_packaged before.
* Add git dependency to package.xml (for Abseil build). (`#1098 <https://github.com/ubica-robotics/cartographer_ros/issues/1098>`_)
* Only include correct source files in cmake (`#1085 <https://github.com/ubica-robotics/cartographer_ros/issues/1085>`_)
  This fixes `#1050 <https://github.com/ubica-robotics/cartographer_ros/issues/1050>`_. Tested standalone compilation and with the debian fakeroot tool. I had to build with a custom Protobuf3 instance though, so another build-check on a regular setup would be appreciated.
  Before this change all *.cc files would be included. If subprojects
  were run individually with the Debian package generator. This resulted in an
  inclusion of temporarily checked out *.cc from the abseil include. This
  change fixes the import behaviour and enables the creation of a valid
  package.
* Remove deleted trajectories from submaps display.  (`#1065 <https://github.com/ubica-robotics/cartographer_ros/issues/1065>`_)
  Fixes strange behavior with empty leftover entries in the trajectory list of the RViz plugin.
  By using an `std::map` instead of an `std::vector` for the trajectories  we can
  easily erase trajectories which do not appear in the submap list anymore.
* Fix submap pose marker toggling. (`#1019 <https://github.com/ubica-robotics/cartographer_ros/issues/1019>`_)
  Set the visibility in he constructor, otherwise the setting won't have
  an effect on newly created submap slices unless it's toggled again.
  Somehow this got lost in `#1012 <https://github.com/ubica-robotics/cartographer_ros/issues/1012>`_, sorry for that.
* Add toggle for pose markers to submaps display RViz plugin. (`#1012 <https://github.com/ubica-robotics/cartographer_ros/issues/1012>`_)
* [ABSL] Use absl::Mutex. (`#969 <https://github.com/ubica-robotics/cartographer_ros/issues/969>`_)
* Follow the Absl update. (`#955 <https://github.com/ubica-robotics/cartographer_ros/issues/955>`_)
* Add .clang-format file. (`#938 <https://github.com/ubica-robotics/cartographer_ros/issues/938>`_)
  This adds a .clang-format file, so that git clang-format uses
  Google style without the need to remember the commandline flag.
  Similar to `googlecartographer/cartographer#1249 <https://github.com/googlecartographer/cartographer/issues/1249>`_.
* set required version of dependencies (`#892 <https://github.com/ubica-robotics/cartographer_ros/issues/892>`_)
* Release 1.0. (`#889 <https://github.com/ubica-robotics/cartographer_ros/issues/889>`_)
* Add maintainers and authors to package.xml (`#886 <https://github.com/ubica-robotics/cartographer_ros/issues/886>`_)
* Fix Clang thread safety guards in drawable_submap.h (`#839 <https://github.com/ubica-robotics/cartographer_ros/issues/839>`_)
  Fixes a Clang compiler error.
* Remove unused variable from submaps_display.cc (`#840 <https://github.com/ubica-robotics/cartographer_ros/issues/840>`_)
* Make fade out distance configurable (`#674 <https://github.com/ubica-robotics/cartographer_ros/issues/674>`_)
* Change link ordering to fix libcartographer_rviz.so. (`#648 <https://github.com/ubica-robotics/cartographer_ros/issues/648>`_)
  This fixes the issue that rviz crashes when loading the cartographer_rviz plugin saying that the symbol `cartographer::io::UnpackTextureData(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, int, int)` is undefined.
  PAIR=@wohe,@SirVer
* Follow `googlecartographer/cartographer#782 <https://github.com/googlecartographer/cartographer/issues/782>`_ (`#633 <https://github.com/ubica-robotics/cartographer_ros/issues/633>`_)
  Towards [RFC06](https://github.com/googlecartographer/rfcs/blob/master/text/0006-serve-ros-map-from-pbstream.md).
  Migrates
  * `FillSubmapSlice` from `pbstream_to_rosmap_main.cc`
  * `SubmapTexture` logics from cartographer_ros
* Contributors: Alexander Belyaev, Guillaume Doisy, Guillaume dev PC, Jihoon Lee, Juraj Oršulić, Matthias Loebach, Michael Grupp, Mikael Arguedas, Sebastian Klose, Steven Palma, Susanne Pielawa, Wolfgang Hess

0.3.0 (2017-11-23)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.2.0...0.3.0

0.2.0 (2017-06-19)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.1.0...0.2.0

0.1.0 (2017-05-18)
------------------
* First unstable development release
