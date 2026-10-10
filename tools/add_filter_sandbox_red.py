from pathlib import Path

path = Path("tests/CMakeLists.txt")
text = path.read_text(encoding="utf-8")
marker = '''add_executable(sandboxPogoJointDemoTests sandbox/pogo_joint_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")
target_include_directories(sandboxPogoJointDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
target_link_libraries(sandboxPogoJointDemoTests PRIVATE zonai::zonai)
add_test(NAME sandboxPogoJointDemoTests COMMAND sandboxPogoJointDemoTests)
'''
block = '''
add_executable(sandboxFilterJointDemoTests sandbox/filter_joint_demo_test.cpp "${PROJECT_SOURCE_DIR}/sandbox/demo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyDemo.cpp" "${PROJECT_SOURCE_DIR}/sandbox/rigidBodyJointDemo.cpp")
target_include_directories(sandboxFilterJointDemoTests PRIVATE "${PROJECT_SOURCE_DIR}/sandbox")
target_link_libraries(sandboxFilterJointDemoTests PRIVATE zonai::zonai)
add_test(NAME sandboxFilterJointDemoTests COMMAND sandboxFilterJointDemoTests)
'''
if "sandboxFilterJointDemoTests" in text:
    raise RuntimeError("Filter sandbox target already exists")
if text.count(marker) != 1:
    raise RuntimeError("Pogo sandbox target marker missing")
path.write_text(text.replace(marker, marker + block, 1), encoding="utf-8")
print("Registered Filter Sandbox RED target")
