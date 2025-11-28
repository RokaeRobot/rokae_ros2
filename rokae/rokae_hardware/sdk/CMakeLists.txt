# Public headers
set(SHARED_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

set(SHARED_HEADERS
    base.h
    data_types.h
    exception.h
    model.h
	robot.h
    motion_control_rt.h
    force_control.h
    planner.h
	utility.h
    )


add_compile_options(-std=c++17)
list(TRANSFORM SHARED_HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/rokae/")
set(SHARED_HEADERS ${SHARED_HEADERS} CACHE INTERNAL "Shared headers with examples")