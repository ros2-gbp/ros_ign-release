%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/rolling/.*$
%global __requires_exclude_from ^/opt/ros/rolling/.*$

Name:           ros-rolling-ros-gz-bridge
Version:        2.1.1
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS ros_gz_bridge package

License:        Apache 2.0
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-rolling-actuator-msgs
Requires:       ros-rolling-geometry-msgs
Requires:       ros-rolling-gps-msgs
Requires:       ros-rolling-gz-msgs-vendor
Requires:       ros-rolling-gz-transport-vendor
Requires:       ros-rolling-launch
Requires:       ros-rolling-launch-ros
Requires:       ros-rolling-nav-msgs
Requires:       ros-rolling-rclcpp
Requires:       ros-rolling-rclcpp-components
Requires:       ros-rolling-ros-gz-interfaces
Requires:       ros-rolling-rosgraph-msgs
Requires:       ros-rolling-sensor-msgs
Requires:       ros-rolling-std-msgs
Requires:       ros-rolling-tf2-msgs
Requires:       ros-rolling-trajectory-msgs
Requires:       ros-rolling-vision-msgs
Requires:       ros-rolling-yaml-cpp-vendor
Requires:       ros-rolling-ros-workspace
BuildRequires:  pkgconfig
BuildRequires:  ros-rolling-actuator-msgs
BuildRequires:  ros-rolling-ament-cmake
BuildRequires:  ros-rolling-ament-cmake-python
BuildRequires:  ros-rolling-geometry-msgs
BuildRequires:  ros-rolling-gps-msgs
BuildRequires:  ros-rolling-gz-msgs-vendor
BuildRequires:  ros-rolling-gz-transport-vendor
BuildRequires:  ros-rolling-launch
BuildRequires:  ros-rolling-launch-ros
BuildRequires:  ros-rolling-nav-msgs
BuildRequires:  ros-rolling-rclcpp
BuildRequires:  ros-rolling-rclcpp-components
BuildRequires:  ros-rolling-ros-gz-interfaces
BuildRequires:  ros-rolling-rosgraph-msgs
BuildRequires:  ros-rolling-rosidl-pycommon
BuildRequires:  ros-rolling-sensor-msgs
BuildRequires:  ros-rolling-std-msgs
BuildRequires:  ros-rolling-tf2-msgs
BuildRequires:  ros-rolling-trajectory-msgs
BuildRequires:  ros-rolling-vision-msgs
BuildRequires:  ros-rolling-yaml-cpp-vendor
BuildRequires:  ros-rolling-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-rolling-ament-cmake-gtest
BuildRequires:  ros-rolling-ament-lint-auto
BuildRequires:  ros-rolling-ament-lint-common
BuildRequires:  ros-rolling-launch-testing
BuildRequires:  ros-rolling-launch-testing-ament-cmake
%endif

%description
Bridge communication between ROS and Gazebo Transport

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/rolling" \
    -DAMENT_PREFIX_PATH="/opt/ros/rolling" \
    -DCMAKE_PREFIX_PATH="/opt/ros/rolling" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/rolling

%changelog
* Mon Oct 14 2024 Aditya Pande <adityapande@intrinsic.ai> - 2.1.1-1
- Autogenerated by Bloom

* Thu Sep 12 2024 Aditya Pande <adityapande@intrinsic.ai> - 2.1.0-1
- Autogenerated by Bloom

* Thu Aug 29 2024 Aditya Pande <adityapande@intrinsic.ai> - 2.0.1-1
- Autogenerated by Bloom

* Mon Jul 22 2024 Aditya Pande <adityapande@intrinsic.ai> - 2.0.0-1
- Autogenerated by Bloom

* Sat Jul 13 2024 Aditya Pande <adityapande@intrinsic.ai> - 1.0.1-2
- Autogenerated by Bloom

* Wed Jul 03 2024 Aditya Pande <adityapande@intrinsic.ai> - 1.0.1-1
- Autogenerated by Bloom

* Wed Apr 24 2024 Aditya Pande <adityapande@intrinsic.ai> - 1.0.0-1
- Autogenerated by Bloom

