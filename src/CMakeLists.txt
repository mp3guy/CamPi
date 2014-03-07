cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

include_directories(/opt/vc/userland/host_applications/linux/libs/bcm_host/include)
include_directories(/opt/vc/userland/interface/vcos)
include_directories(/opt/vc/userland)
include_directories(/opt/vc/userland/interface/vcos/pthreads)
include_directories(/opt/vc/userland/interface/vmcs_host/linux)

set(CMAKE_C_FLAGS "-Wall -O3")

add_executable(raspiyuv 
               RaspiCamControl.c
               RaspiCLI.c
               RaspiPreview.c 
               RaspiStillYUV.c)

configure_file(run.sh ${CMAKE_CURRENT_BINARY_DIR}/run.sh COPYONLY)
configure_file(cam.sh ${CMAKE_CURRENT_BINARY_DIR}/cam.sh COPYONLY)

target_link_libraries(raspiyuv 
                      /opt/vc/userland/lib/libmmal_core.so 
                      /opt/vc/userland/lib/libmmal_util.so 
                      /opt/vc/userland/lib/libmmal_vc_client.so 
                      /opt/vc/userland/lib/libvcos.so 
                      /opt/vc/userland/lib/libbcm_host.so
                      ${OpenCV_LIBS}
                      pthread)
