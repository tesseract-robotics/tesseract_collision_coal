find_package(octomap REQUIRED)
find_package(coal REQUIRED)

find_package(boost_plugin_loader REQUIRED)
find_package(tesseract_collision)
find_package(tesseract_geometry)

# Create target for Coal implementation
add_library(
  ${PROJECT_NAME}_coal
  src/coal_cast_managers.cpp
  src/coal_casthullshape_utility.cpp
  src/coal_casthullshape.cpp
  src/coal_collision_object_wrapper.cpp
  src/coal_discrete_managers.cpp
  src/coal_utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}_coal
  PUBLIC Eigen3::Eigen
         boost_plugin_loader::boost_plugin_loader
         coal::coal
         console_bridge::console_bridge
         octomap
         octomath
         tesseract::tesseract_collision_core
         tesseract::tesseract_geometry)
target_compile_options(${PROJECT_NAME}_coal PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_coal PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_coal PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_coal PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_coal ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_coal
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_coal PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

add_library(${PROJECT_NAME}_coal_factories src/coal_factories.cpp)
target_link_libraries(${PROJECT_NAME}_coal_factories PUBLIC ${PROJECT_NAME}_coal)
target_compile_options(${PROJECT_NAME}_coal_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_coal_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_coal_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_coal_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_coal_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_coal_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_coal_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                        "$<INSTALL_INTERFACE:include>")

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "${PROJECT_NAME}_coal_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT coal
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(
  TARGETS ${PROJECT_NAME}_coal_factories
  COMPONENT coal
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT coal
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_coal
  DEPENDENCIES "tesseract_collision COMPONENTS core" "coal")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT coal
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision Coal components"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS "coal | ${TESSERACT_PACKAGE_PREFIX}coal"
    WINDOWS_DEPENDS "coal | ${TESSERACT_PACKAGE_PREFIX}coal")
endif()
