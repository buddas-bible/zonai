from pathlib import Path

world_h = Path('src/dynamics/world.h')
h = world_h.read_text(encoding='utf-8')
old_decl = '''    // 연결된 shape들의 density / geometry를 합산해 Dynamic body의 mass data를 갱신함.\n    void UpdateBodyMassData( std::int32_t bodyIndex );\n'''
new_decl = '''    // 현재 local center 기준으로 모든 shape의 CCD / sleep extent만 다시 합산함.\n    // 질량과 COM은 바꾸지 않음.\n    void UpdateBodyShapeExtents( std::int32_t bodyIndex );\n\n    // 연결된 shape들의 density / geometry를 합산해 Dynamic body의 mass data를 갱신함.\n    void UpdateBodyMassData( std::int32_t bodyIndex );\n'''
if old_decl not in h:
    raise SystemExit('world.h extent declaration anchor not found')
h = h.replace(old_decl, new_decl, 1)
world_h.write_text(h, encoding='utf-8')

world_cpp = Path('src/dynamics/world.cpp')
cpp = world_cpp.read_text(encoding='utf-8')
function_anchor = 'void world::UpdateBodyMassData( std::int32_t bodyIndex )\n{'
helper = '''void world::UpdateBodyShapeExtents( std::int32_t bodyIndex )
{
    assert( bodyIndex >= 0 );
    assert( static_cast<std::size_t>( bodyIndex ) < bodies_.size() );
    assert( bodySims_.size() == bodies_.size() );

    const body& body = bodies_[bodyIndex];
    bodySim& bodySim = bodySims_[bodyIndex];

    assert( body.bodyId == bodyIndex );
    assert( bodySim.bodyId == bodyIndex );

    bodySim.minExtent = std::numeric_limits<float>::max();
    bodySim.maxExtent = 0.0f;

    std::int32_t shapeIndex = body.headShapeId;
    std::int32_t visitedCount = 0;

    while( shapeIndex != body::NULL_INDEX )
    {
        assert( shapeIndex >= 0 );
        assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );
        assert( visitedCount < body.shapeCount );

        const shape& shape = shapes_[shapeIndex];
        assert( shape.bodyId == bodyIndex );

        const shapeExtent2 extent = ComputeShapeExtent( shape.geometry, bodySim.localCenter );
        bodySim.minExtent = std::min( bodySim.minExtent, extent.minExtent );
        bodySim.maxExtent = std::max( bodySim.maxExtent, extent.maxExtent );

        shapeIndex = shape.nextShapeId;
        ++visitedCount;
    }

    assert( visitedCount == body.shapeCount );
}

'''
if helper not in cpp:
    if function_anchor not in cpp:
        raise SystemExit('UpdateBodyMassData anchor not found')
    cpp = cpp.replace(function_anchor, helper + function_anchor, 1)

cpp = cpp.replace('''    bodySim.localCenter = {};\n    bodySim.minExtent = std::numeric_limits<float>::max();\n    bodySim.maxExtent = 0.0f;\n\n    const auto updateExtents = [&]()\n    {\n        std::int32_t shapeIndex = body.headShapeId;\n        std::int32_t visitedCount = 0;\n\n        while( shapeIndex != body::NULL_INDEX )\n        {\n            assert( shapeIndex >= 0 );\n            assert( static_cast<std::size_t>( shapeIndex ) < shapes_.size() );\n            assert( visitedCount < body.shapeCount );\n\n            const shape& shape = shapes_[shapeIndex];\n\n            assert( shape.bodyId == bodyIndex );\n\n            const shapeExtent2 extent = ComputeShapeExtent( shape.geometry, bodySim.localCenter );\n\n            bodySim.minExtent = std::min( bodySim.minExtent, extent.minExtent );\n\n            bodySim.maxExtent = std::max( bodySim.maxExtent, extent.maxExtent );\n\n            shapeIndex = shape.nextShapeId;\n            ++visitedCount;\n        }\n\n        assert( visitedCount == body.shapeCount );\n    };\n''', '''    bodySim.localCenter = {};\n''', 1)
cpp = cpp.replace('        updateExtents();\n        return;', '        UpdateBodyShapeExtents( bodyIndex );\n        return;', 1)
cpp = cpp.replace('    updateExtents();\n\n    // local center of mass', '    UpdateBodyShapeExtents( bodyIndex );\n\n    // local center of mass', 1)
world_cpp.write_text(cpp, encoding='utf-8')

chain_cpp = Path('src/dynamics/chainSegmentWorld.cpp')
chain = chain_cpp.read_text(encoding='utf-8')
old_chain = '''    storedShape.geometry = chainSegment;\n    storedShape.aabbMargin = ComputeShapeAABBMargin( storedShape.geometry );\n\n    const transform2 transform = bodySims_[bodyIndex].transform;\n'''
new_chain = '''    storedShape.geometry = chainSegment;\n    storedShape.aabbMargin = ComputeShapeAABBMargin( storedShape.geometry );\n\n    // Chain은 질량/COM을 바꾸지 않지만 local center에서의 최대 반경은\n    // CCD, sleep, contact recycling 판정에 사용되므로 geometry 교체 직후 다시 합산함.\n    UpdateBodyShapeExtents( bodyIndex );\n\n    const transform2 transform = bodySims_[bodyIndex].transform;\n'''
if old_chain not in chain:
    raise SystemExit('chain segment extent call anchor not found')
chain = chain.replace(old_chain, new_chain, 1)
chain_cpp.write_text(chain, encoding='utf-8')
