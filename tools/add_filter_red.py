from pathlib import Path

path = Path("tests/dynamics/pogo_joint_world_test.cpp")
text = path.read_text(encoding="utf-8")
marker = "    return EXIT_SUCCESS;\n}"
block = '''    // Filter Joint는 solver 없이 두 Body 사이 collision만 차단하고 공용 Joint lifecycle에 참여함.\n    {\n        world filterWorld;\n        filterWorld.SetGravity( {} );\n        const bodyId filterA = filterWorld.CreateBody( bodyType::Dynamic );\n        const bodyId filterB = filterWorld.CreateBody( bodyType::Dynamic );\n        ( void )filterWorld.CreateShape( filterA, circle2{ {}, 0.5f } );\n        ( void )filterWorld.CreateShape( filterB, circle2{ {}, 0.5f } );\n        filterWorld.Step( h, 1 );\n        check( filterWorld.GetContactCount() == 1, "Filter RED starts with an overlapping Contact" );\n\n        filterJointDef filterDefinition{};\n        filterDefinition.bodyA = filterA;\n        filterDefinition.bodyB = filterB;\n        const jointId filter = filterWorld.createFilterJoint( filterDefinition );\n        check( filterWorld.GetContactCount() == 0, "Filter creation removes existing contacts immediately" );\n        check( !filterWorld.getFilterJointData( filter ).collideConnected, "Filter disables connected collision by default" );\n    }\n\n'''
if "Filter RED starts with an overlapping Contact" in text:
    raise RuntimeError("Filter RED already applied")
if text.count(marker) != 1:
    raise RuntimeError("Pogo test return marker missing")
path.write_text(text.replace(marker, block + marker, 1), encoding="utf-8")
print("Added Filter Joint RED contract")
