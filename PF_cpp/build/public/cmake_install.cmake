# Install script for directory: /Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/public/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysclass/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysclass11/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysport/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgl/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysglcpp/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysglcpp/src/gl2/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysglcpp/src/gl1/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysglcpp/src/nownd/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysbitmap/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysbitmap/cmdtools/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysbitmaputil/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysbitmapfont/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysbitmapfont/cmdtools/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/yssystemfont/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fssimplewindow/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fssimplewindow/template/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fssimplewindow/samples/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/yssimplesound/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/yssimplesound/test/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/yssimplesound/sample/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/yssimplesound/sample_fssimplewindow/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fslazywindow/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fslazywindow/template/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fslazywindow/samples/sample00-bouncingBall/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fssimplefiledialog/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fssimplefiledialog/sample/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysfontrenderer/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ystexturemanager/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ystexturemanager/src/gl/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ystexturemanager/src/null/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsguilib/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsguilib/src/gl2/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsguilib/src/gl1/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsguilib/src/nownd/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsguilib/filedialog/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsguilib/template/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsguilib/template_with_filedialog/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsgui3d/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsgui3d/src/gl1/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsgui3d/src/gl2/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsgui3d/template/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsgui3d/template_with_filedialog/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/fsgui3d/template-easy/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/yssocket/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/kernel/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/kernelutil/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/shellrender/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/shellrender/gl2/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/shellrender/gl1/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/shellrender/nownd/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/gui_foundation/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/gui/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/main/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/cmdlib/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/cmdmain/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ysgebl/src/samples/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/2DRetroMapTool/src/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/ir_transceiver/cmake_install.cmake")
  include("/Users/jaeseoko/Desktop/COURSES/SS21/16833_LocMap/Project/16833-project/build/public/opencv/cmake_install.cmake")

endif()

