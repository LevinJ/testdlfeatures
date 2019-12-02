cmake_minimum_required (VERSION 2.6)

project (testdlfeatures)

set(Torch_DIR "/home/levin/workspace/tools/pytorch/torch/lib/tmp_install/share/cmake/Torch")  # libtorch
find_package(OpenCV 3.4 QUIET)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Torch REQUIRED)
message("Using version ${OpenCV_VERSION}")


add_executable(testdlfeatures testdlfeatures.cpp SuperPoint.cc SPextractor.cc)

target_link_libraries(${PROJECT_NAME}
${TORCH_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
)
