from pathlib import Path

path = Path('tests/dynamics/body_shape_contact_lifecycle_test.cpp')
text = path.read_text(encoding='utf-8')
marker = '''    check( !simulation.IsValid( oldContact ), "orphan chain old contact resurrected" );\n\n    const std::array<vec2, 2> points'''
block = '''    check( !simulation.IsValid( oldContact ), "orphan chain old contact resurrected" );\n\n    // Geometry가 body local center에서 멀어지면 CCD / sleep에서 쓰는 maxExtent도 즉시 갱신되어야 함.\n    world extentSimulation{};\n    bodyDef extentBodyDefinition{};\n    extentBodyDefinition.type = bodyType::Dynamic;\n    extentBodyDefinition.gravityScale = 0.0f;\n    extentBodyDefinition.angularVelocity = 0.01f;\n    extentBodyDefinition.sleepThreshold = 0.05f;\n    const bodyId extentBody = extentSimulation.CreateBody( extentBodyDefinition );\n\n    const chainSegment2 nearCenter{\n        { 0.2f, 0.0f },\n        { { 0.1f, 0.0f }, { -0.1f, 0.0f } },\n        { -0.2f, 0.0f }\n    };\n    const shapeId extentShape = extentSimulation.CreateShape( extentBody, nearCenter, {}, 0.0f );\n\n    const chainSegment2 farFromCenter{\n        { 11.2f, 0.0f },\n        { { 11.1f, 0.0f }, { 10.9f, 0.0f } },\n        { 10.8f, 0.0f }\n    };\n    extentSimulation.SetChainSegment( extentShape, farFromCenter );\n\n    for( int i = 0; i < 40; ++i )\n    {\n        extentSimulation.Step( 1.0f / 60.0f, 1 );\n    }\n\n    check( extentSimulation.IsBodyAwake( extentBody ), "orphan chain update kept stale body extent" );\n\n    const std::array<vec2, 2> points'''
if marker not in text:
    raise SystemExit('extent regression anchor not found')
path.write_text(text.replace(marker, block, 1), encoding='utf-8')
