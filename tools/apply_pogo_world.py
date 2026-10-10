from pathlib import Path

path = Path("tests/CMakeLists.txt")
text = path.read_text(encoding="utf-8")
marker = '''add_executable(moverJointWorldTests dynamics/mover_joint_world_test.cpp)\ntarget_link_libraries(moverJointWorldTests PRIVATE zonai::zonai)\nadd_test(NAME moverJointWorldTests COMMAND moverJointWorldTests)\n'''
insert = marker + '''\nadd_executable(pogoJointTests dynamics/pogo_joint_test.cpp)\ntarget_link_libraries(pogoJointTests PRIVATE zonai::zonai)\nadd_test(NAME pogoJointTests COMMAND pogoJointTests)\n\nadd_executable(pogoJointWorldTests dynamics/pogo_joint_world_test.cpp)\ntarget_link_libraries(pogoJointWorldTests PRIVATE zonai::zonai)\nadd_test(NAME pogoJointWorldTests COMMAND pogoJointWorldTests)\n'''
count = text.count(marker)
if count != 1:
    raise RuntimeError(f"expected one Mover test marker, found {count}")
path.write_text(text.replace(marker, insert, 1), encoding="utf-8")
print("Registered Pogo solver and World tests")
