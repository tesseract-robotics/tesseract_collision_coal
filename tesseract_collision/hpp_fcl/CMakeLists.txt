find_package(octomap REQUIRED)
find_package(hpp-fcl REQUIRED)

find_package(tesseract_collision)
find_package(tesseract_geometry)

# Create target for HPP-FCL implementation
add_library(${PROJECT_NAME}_hpp_fcl src/hpp_fcl_discrete_managers.cpp src/hpp_fcl_utils.cpp
                                    src/hpp_fcl_collision_object_wrapper.cpp)
target_link_libraries(
  ${PROJECT_NAME}_hpp_fcl
  PUBLIC Eigen3::Eigen
         tesseract::tesseract_geometry
         hpp-fcl::hpp-fcl
         console_bridge::console_bridge
         octomap
         octomath
         tesseract::tesseract_collision_core
         tesseract::tesseract_geometry
         )
target_compile_options(${PROJECT_NAME}_hpp_fcl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_hpp_fcl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_hpp_fcl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_hpp_fcl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_hpp_fcl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_hpp_fcl
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_hpp_fcl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

add_library(${PROJECT_NAME}_hpp_fcl_factories src/hpp_fcl_factories.cpp)
target_link_libraries(${PROJECT_NAME}_hpp_fcl_factories PUBLIC ${PROJECT_NAME}_hpp_fcl)
target_compile_options(${PROJECT_NAME}_hpp_fcl_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_hpp_fcl_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_hpp_fcl_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_hpp_fcl_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_hpp_fcl_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_hpp_fcl_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_hpp_fcl_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                           "$<INSTALL_INTERFACE:include>")

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "${PROJECT_NAME}_hpp_fcl_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT hpp_fcl
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(
  TARGETS ${PROJECT_NAME}_hpp_fcl_factories
  COMPONENT hpp_fcl
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT hpp_fcl
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_hpp_fcl
  DEPENDENCIES "tesseract_collision COMPONENTS core" "hpp-fcl")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT hpp_fcl
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision HPP-FCL components"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS "hpp-fcl | ${TESSERACT_PACKAGE_PREFIX}hpp_fcl"
    WINDOWS_DEPENDS "hpp-fcl | ${TESSERACT_PACKAGE_PREFIX}hpp_fcl")
endif()
