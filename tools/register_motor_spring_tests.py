from pathlib import Path

path = Path("tests/CMakeLists.txt")
text = path.read_text(encoding="utf-8")
marker = """add_executable(motorJointTests dynamics/motor_joint_test.cpp)\ntarget_link_libraries(motorJointTests PRIVATE zonai::zonai)\nadd_test(NAME motorJointTests COMMAND motorJointTests)\n"""
replacement = marker + """\nadd_executable(motorJointSpringTests dynamics/motor_joint_spring_test.cpp)\ntarget_link_libraries(motorJointSpringTests PRIVATE zonai::zonai)\nadd_test(NAME motorJointSpringTests COMMAND motorJointSpringTests)\n\nadd_executable(motorJointSpringWorldTests dynamics/motor_joint_spring_world_test.cpp)\ntarget_link_libraries(motorJointSpringWorldTests PRIVATE zonai::zonai)\nadd_test(NAME motorJointSpringWorldTests COMMAND motorJointSpringWorldTests)\n"""

if "motorJointSpringTests" not in text:
    if marker not in text:
        raise SystemExit("Motor Joint test marker not found")
    text = text.replace(marker, replacement, 1)
    path.write_text(text, encoding="utf-8")
