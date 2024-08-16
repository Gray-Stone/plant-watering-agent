
https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/

However this won't work if also defining interfaces.

```
CMake Error at /opt/ros/humble/share/ament_cmake_python/cmake/ament_python_install_package.cmake:106 (add_custom_target):
  add_custom_target cannot create target
  "ament_cmake_python_symlink_stretch_mover" because another target with the
  same name already exists.  The existing target is a custom target created
  in source directory
```

Will get this error when building