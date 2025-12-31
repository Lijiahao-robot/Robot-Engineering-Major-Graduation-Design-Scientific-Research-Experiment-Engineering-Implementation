find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
target_link_libraries(fault_diagnosis_node ${YAML_CPP_LIBRARIES})
