cmake_minimum_required(VERSION 2.8.9)
# before openrave
add_subdirectory(ann)
add_subdirectory(zlib)
add_subdirectory(minizip)
add_subdirectory(fparser)
add_subdirectory(pcre)

add_subdirectory(convexdecomposition)
add_subdirectory(crlibm)
# add_subdirectory(flann)
add_subdirectory(ivcon)
add_subdirectory(qhull)

# after openrave
# add_subdirectory(collada)
