#
# Copyright Projet Lagadic / Inria Rennes, 2015
# www: http://www.irisa.fr/lagadic
#
# Author: Alexandre Krupa email:Alexandre.Krupa@inria.fr
#
# SOURCE variable corresponds to the list of all the sources to build binaries.
# The generate binary comes by removing the .cpp extension to
# the source name.
#
# If you want to add/remove a source, modify here
set(SOURCES
servo-simu-dense.cpp
usImageGradient.cpp
usIntensityInteraction.cpp
)

set(HEADERS
usImageGradient.h
usIntensityInteraction.h
vpCurvePlotter.h
)


# rule for binary build

# From source compile the binary and add link rules
add_executable(servo-simu-dense ${SOURCES} ${HEADERS})    
target_link_libraries(servo-simu-dense ${UltrasoundDense_EXTERN_LIBS}) 

