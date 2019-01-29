from conans import ConanFile, CMake
from conans import tools
from conans.tools import os_info, SystemPackageTool
import os, sys
import sysconfig
from io import StringIO


class DeviceCameraZedConan(ConanFile):
    name = "ubitrack_device_camera_zed"
    version = "1.3.0"

    description = "Ubitrack ZED Camera Capture"
    url = "https://github.com/Ubitrack/device_camera_zed.git"
    license = "GPL"
    options = {
        "zedsdk_version": ["2.7.1", ],
        "zedsdk_root": "ANY",
        }
    default_options = (
        "zedsdk_version=2.7.1",
        "zedsdk_root=/usr/local/zed",
        )

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"
    requires = (
        "ubitrack_core/%s@ubitrack/stable" % version,
        "ubitrack_vision/%s@ubitrack/stable" % version,
        "ubitrack_dataflow/%s@ubitrack/stable" % version,
        "opencv/[>=3.2.0]@camposs/stable", 
        # "cuda_dev_config/[>=1.0]@camposs/stable",
       )

    # all sources are deployed with the package
    exports_sources = "cmake/*", "include/*", "doc/*", "lib/*", "src/*", "CMakeLists.txt"

    #pull zed-sdk libs
    def imports(self):
        self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
        self.copy(pattern="*.dylib*", dst="lib", src="lib") 
        self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        cmake.verbose = True
        # cmake.definitions['CUDA_TOOLKIT_ROOT_DIR'] = self.deps_user_info["cuda_dev_config"].cuda_root
        cmake.definitions['ZED_ROOT'] = self.options.zedsdk_root
        cmake.configure()
        cmake.build()
        cmake.install()
