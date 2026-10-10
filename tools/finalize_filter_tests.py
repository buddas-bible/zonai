from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected 1 match, got {count}")
    return text.replace(old, new, 1)

cmake_path = Path("tests/CMakeLists.txt")
cmake = cmake_path.read_text(encoding="utf-8")
pogo_block = '''add_executable(pogoJointWorldTests dynamics/pogo_joint_world_test.cpp)\ntarget_link_libraries(pogoJointWorldTests PRIVATE zonai::zonai)\nadd_test(NAME pogoJointWorldTests COMMAND pogoJointWorldTests)\n'''
filter_block = '''\nadd_executable(filterJointTests dynamics/filter_joint_test.cpp)\ntarget_link_libraries(filterJointTests PRIVATE zonai::zonai)\nadd_test(NAME filterJointTests COMMAND filterJointTests)\n\nadd_executable(filterJointWorldTests dynamics/filter_joint_world_test.cpp)\ntarget_link_libraries(filterJointWorldTests PRIVATE zonai::zonai)\nadd_test(NAME filterJointWorldTests COMMAND filterJointWorldTests)\n'''
cmake = replace_once(cmake, pogo_block, pogo_block + filter_block, "Filter CMake targets")
cmake_path.write_text(cmake, encoding="utf-8")

pogo_path = Path("tests/dynamics/pogo_joint_world_test.cpp")
pogo = pogo_path.read_text(encoding="utf-8")n