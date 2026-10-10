from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count < 1:
        raise RuntimeError(f"{label}: marker missing")
    return text.replace(old, new, 1)


# Constructor dispatch.
path = Path("sandbox/rigidBodyDemo.cpp")
text = path.read_text(encoding="utf-8")
old = '''    else if( kind == demoKind::pogoJointPlayground )
    {
        createPogoJointPlayground();
    }
    else
    {
        assert( kind == demoKind::motorCar );
'''
new = '''    else if( kind == demoKind::pogoJointPlayground )
    {
        createPogoJointPlayground();
    }
    else if( kind == demoKind::filterJointPlayground )
    {
        createFilterJointPlayground();
    }
    else
    {
        assert( kind == demoKind::motorCar );
'''
text = replace_once(text, old, new, "rigidBodyDemo constructor")
path.write_text(text, encoding="utf-8")

# Filter runtime toggle and scene creation.
path = Path("sandbox/rigidBodyJointDemo.cpp")
text = path.read_text(encoding="utf-8")
setting_marker = '''void rigidBodyDemo::setPogoJointSettings( float restLength, float hertz, float dampingRatio, float maxTensionForce, float maxCompressionForce )
{
    if( kind_ != demoKind::pogoJointPlayground || !world_.IsValid( pogoJoint_ ) ) return;
    world_.setPogoJointSpring( pogoJoint_, restLength, hertz, dampingRatio );
    world_.setPogoJointForceLimits( pogoJoint_, maxTensionForce, maxCompressionForce );
}
'''
setting_block = setting_marker + '''
void rigidBodyDemo::setFilterJointEnabled( bool enabled )
{
    if( kind_ != demoKind::filterJointPlayground ) return;

    if( enabled )
    {
        if( world_.IsValid( filterJoint_ ) ) return;

        filterJointDef joint{};
        joint.bodyA = impulseBody_;
        joint.bodyB = torqueBody_;
        filterJoint_ = world_.createFilterJoint( joint );
    }
    else
    {
        if( !world_.IsValid( filterJoint_ ) ) return;

        world_.destroyJoint( filterJoint_ );
        filterJoint_ = {};
    }

    // Joint 생성은 기존 Contact를 즉시 제거하고, 파괴는 proxy를 touch해 같은 pair를 다시 검사함.
    refreshContacts();
}
'''
text = replace_once(text, setting_marker, setting_block, "Filter Sandbox toggle")

scene_marker = 'void rigidBodyDemo::createPogoJointPlayground()\n'
scene_block = '''void rigidBodyDemo::createFilterJointPlayground()
{
    world_.SetGravity( {} );

    // 두 원을 겹쳐 놓아 Filter On/Off만으로 collision 차이가 즉시 보이게 함.
    impulseBody_ = world_.CreateBody( bodyType::Dynamic, { { -0.35f, 0.0f }, {} } );
    torqueBody_ = world_.CreateBody( bodyType::Dynamic, { { 0.35f, 0.0f }, {} } );

    const shapeId shapeA = world_.CreateShape( impulseBody_, circle2{ {}, 0.6f } );
    const shapeId shapeB = world_.CreateShape( torqueBody_, circle2{ {}, 0.6f } );
    shapes_.push_back( { impulseBody_, shapeA, "A [Filter 대상]" } );
    shapes_.push_back( { torqueBody_, shapeB, "B [Filter 대상]" } );

    filterJointDef joint{};
    joint.bodyA = impulseBody_;
    joint.bodyB = torqueBody_;
    filterJoint_ = world_.createFilterJoint( joint );
}

'''
text = replace_once(text, scene_marker, scene_block + scene_marker, "Filter Sandbox scene")
path.write_text(text, encoding="utf-8")

# ImGui controls and debug draw.
path = Path("sandbox/jointDemoView.cpp")
text = path.read_text(encoding="utf-8")
old_gate = 'if( kind != demoKind::distancePendulum && kind != demoKind::revoluteHinge && kind != demoKind::wheelSuspension && kind != demoKind::prismaticRail && kind != demoKind::weldPair && kind != demoKind::mouseJointPlayground && kind != demoKind::motorJointPlayground && kind != demoKind::moverJointPlayground && kind != demoKind::pogoJointPlayground ) return;'
new_gate = 'if( kind != demoKind::distancePendulum && kind != demoKind::revoluteHinge && kind != demoKind::wheelSuspension && kind != demoKind::prismaticRail && kind != demoKind::weldPair && kind != demoKind::mouseJointPlayground && kind != demoKind::motorJointPlayground && kind != demoKind::moverJointPlayground && kind != demoKind::pogoJointPlayground && kind != demoKind::filterJointPlayground ) return;'
text = replace_once(text, old_gate, new_gate, "Filter view gate")

pogo_branch = '            else if( kind == demoKind::pogoJointPlayground )\n'
filter_branch = '''            else if( kind == demoKind::filterJointPlayground )
            {
                bool enabled = view_.getWorld().IsValid( view_.getFilterJoint() );
                if( ImGui::Checkbox( "Filter Enabled###FilterJointEnabled", &enabled ) ) view_.setFilterJointEnabled( enabled );
                ImGui::TextWrapped( "Filter는 solver 힘을 만들지 않고 이 두 Body 사이 Contact만 차단합니다. Off는 Joint를 파괴해 같은 pair의 collision을 다시 허용합니다." );
                ImGui::Text( "Contacts: %zu", view_.getWorld().GetContactCount() );
            }
'''
text = replace_once(text, pogo_branch, filter_branch + pogo_branch, "Filter quick settings")

draw_marker = '''        view_.draw( draw );
        if( view_.getKind() == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )
'''
draw_block = '''        view_.draw( draw );
        if( view_.getKind() == demoKind::filterJointPlayground )
        {
            if( view_.getWorld().IsValid( view_.getFilterJoint() ) )
            {
                const filterJointData joint = view_.getWorld().getFilterJointData( view_.getFilterJoint() );
                const vec2 centerA = TransformPoint( view_.getWorld().GetBodyTransform( joint.bodyA ), view_.getWorld().GetBodyLocalCenter( joint.bodyA ) );
                const vec2 centerB = TransformPoint( view_.getWorld().GetBodyTransform( joint.bodyB ), view_.getWorld().GetBodyLocalCenter( joint.bodyB ) );
                constexpr ImU32 BODY_COLOR = IM_COL32( 100, 235, 220, 255 );
                constexpr ImU32 FILTER_COLOR = IM_COL32( 255, 190, 70, 255 );
                draw.DrawPoint( centerA, BODY_COLOR, 6.0f );
                draw.DrawPoint( centerB, BODY_COLOR, 6.0f );
                draw.DrawSegment( { centerA, centerB }, FILTER_COLOR, 2.0f );
            }
            return;
        }

        if( view_.getKind() == demoKind::motorJointPlayground && view_.getWorld().IsValid( view_.getMotorJoint() ) )
'''
text = replace_once(text, draw_marker, draw_block, "Filter debug draw")
path.write_text(text, encoding="utf-8")

# Demo-count regressions all describe the same selectable registry size.
for filename in [
    "tests/sandbox/demo_test.cpp",
    "tests/sandbox/joint_demo_test.cpp",
    "tests/sandbox/motor_joint_demo_test.cpp",
    "tests/sandbox/mover_joint_demo_test.cpp",
    "tests/sandbox/pogo_joint_demo_test.cpp",
]:
    path = Path(filename)
    text = path.read_text(encoding="utf-8")
    old = "getDemoEntries().size() == 11"
    if text.count(old) != 1:
        raise RuntimeError(f"{filename}: expected one 11-demo assertion")
    path.write_text(text.replace(old, "getDemoEntries().size() == 12", 1), encoding="utf-8")

print("Applied Filter Joint Sandbox model, controls, draw, and demo counts")
