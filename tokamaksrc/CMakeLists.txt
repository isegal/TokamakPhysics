set(SOURCE_FILES
        src/boxcylinder.cpp
        src/collision.cpp
        src/collisionbody.cpp
        src/constraint.cpp
        src/cylinder.cpp
        src/dcd.cpp
        src/lines.cpp
        src/ne_interface.cpp
        src/perflinux.cpp
        src/region.cpp
        src/restcontact.cpp
        src/rigidbody.cpp
        src/rigidbodybase.cpp
        src/scenery.cpp
        src/simulator.cpp
        src/solver.cpp
        src/sphere.cpp
        src/stack.cpp
        src/tricollision.cpp
        )

add_library(tokamaksrc ${SOURCE_FILES})
target_include_directories(tokamaksrc
        PUBLIC ../include
        PRIVATE src)

