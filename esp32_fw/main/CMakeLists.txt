idf_component_register(SRCS "app_main.cpp" "ros2node.cpp" "cam.cpp" "console.cpp" "gps.cpp" "i2chandler.cpp" "webui.cpp" "wifi.cpp" "i2c_updater.cpp" "libnmea/nmea/nmea.c" "libnmea/nmea/parser_static.c" "libnmea/parsers/gpgga.c" "libnmea/parsers/parse.c"  INCLUDE_DIRS . ulp)
set(COMPONENT_REQUIRES ulp server websocket micro-ROS libnmea/nmea libnmea/parsers )

add_compile_definitions(PARSER_COUNT=1 ENABLE_GPGGA=1)

include_directories(libnmea/nmea libnmea/parsers)

#"libnmea/nmea/parser.c" 
#main/libnmea/nmea/nmea.c  main/libnmea/nmea/parser.c  main/libnmea/nmea/parser_static.c
#main/libnmea/parsers/gpgga.c  main/libnmea/parsers/gpgsa.c  main/libnmea/parsers/gprmc.c  main/libnmea/parsers/gpvtg.c
#main/libnmea/parsers/gpgll.c  main/libnmea/parsers/gpgsv.c  main/libnmea/parsers/gptxt.c  main/libnmea/parsers/parse.c

#
# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be unique (if multiple components use ULP).
##set(ulp_app_name ulp_${COMPONENT_NAME})
#
# 2. Specify all assembly source files.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCS.
##set(ulp_s_sources "ulp/adc.S")
#
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
##set(ulp_exp_dep_srcs "app_main.c")
#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
##ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
