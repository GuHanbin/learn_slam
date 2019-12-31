include_directories(${DIRS})
SET( EXAMPLE_SOURCES learn_so3 GaussNewton ceresCurveFitting g2oCurveFitting)

FOREACH(example_src ${EXAMPLE_SOURCES})
  add_executable(${example_src} ${example_src}.cpp)
  target_link_libraries(${example_src} ${LIBS})
ENDFOREACH(example_src)