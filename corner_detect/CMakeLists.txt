cmake_minimum_required(VERSION 2.8)
project( Ex2 )

MESSAGE( STATUS "------------------" ${OpenCV_INCLUDE_DIRS} )

#
find_package( OpenCV REQUIRED )

#
add_executable( Harris
	Harris.cpp

	main.cpp
)


target_include_directories( Harris
	PRIVATE 	.
			${OpenCV_INCLUDE_DIRS}
)

target_link_libraries( Harris 
	${OpenCV_LIBS} 
)
