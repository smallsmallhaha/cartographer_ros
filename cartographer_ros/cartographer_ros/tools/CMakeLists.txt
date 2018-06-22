google_binary(cartographer_parse_pbstream_file
  SRCS
    _parse_pbstream_file.cc
)

install(TARGETS cartographer_parse_pbstream_file
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
