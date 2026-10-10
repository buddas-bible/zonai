from pathlib import Path

path = Path("tests/CMakeLists.txt")
text = path.read_text(encoding="utf-8")
marker = '''add_executable(sandboxMoverJointDemoTests sandbox/mover_joint_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")\ntarget_include_directories(sandboxMoverJointDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")\ntarget_link_libraries(sandboxMoverJointDemoTests PRIVATE zonai::zonai)\nadd_test(NAME sandboxMoverJointDemoTests COMMAND sandboxMoverJointDemoTests)\n'''
insert = marker + '''\nadd_executable(sandboxPogoJointDemoTests sandbox/pogo_joint_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")\ntarget_include_directories(sandboxPogoJointDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")\ntarget_link_libraries(sandboxPogoJointDemoTests PRIVATE zonai::zonai)\nadd_test(NAME sandboxPogoJointDemoTests COMMAND sandboxPogoJointDemoTests)\n'''
count = text.count(marker)
if count != 1:
    raise RuntimeError(f"expected one Mover Sandbox test marker, found {count}")
path.write_text(text.replace(marker, insert, 1), encoding="utf-8")
print("Registered Pogo Sandbox test")
