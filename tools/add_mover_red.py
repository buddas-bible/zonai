from pathlib import Path

path = Path("tests/sandbox/joint_demo_ui_test.cpp")
text = path.read_text(encoding="utf-8")
marker = '''    if( weldVertices <= 0 ) fail( "Weld joint view drew no anchor or frame overlay" );

    ImGui::DestroyContext();
'''
block = '''    if( weldVertices <= 0 ) fail( "Weld joint view drew no anchor or frame overlay" );

    // 같은 Mover scene의 기본 controls와 Joint wrapper controls를 비교함.
    // 구현 전에는 jointDemoView가 공통 rigidBodyDemo controls만 그리고 바로 return하므로 높이가 같아야 함.
    auto moverBase = createRigidBodyDemo( demoKind::moverJointPlayground );
    auto moverView = createJointDemoView( demoKind::moverJointPlayground );

    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
    ImGui::SetNextWindowSize( { 420.0f, 1400.0f } );
    ImGui::Begin( "Mover base controls" );
    const float moverBaseBefore = ImGui::GetCursorPosY();
    moverBase->drawControls();
    const float moverBaseHeight = ImGui::GetCursorPosY() - moverBaseBefore;
    ImGui::End();

    ImGui::SetNextWindowPos( { 440.0f, 0.0f } );
    ImGui::SetNextWindowSize( { 420.0f, 1400.0f } );
    ImGui::Begin( "Mover joint controls" );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "JointQuickSettings" ), 1 );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "MoverJointInspector" ), 1 );
    const float moverViewBefore = ImGui::GetCursorPosY();
    moverView->drawControls();
    const float moverViewHeight = ImGui::GetCursorPosY() - moverViewBefore;
    ImGui::End();
    ImGui::Render();

    if( moverViewHeight <= moverBaseHeight ) fail( "Mover joint view added no joint controls" );

    ImGui::DestroyContext();
'''
if block not in text:
    if marker not in text:
        raise RuntimeError("Mover UI RED marker not found")
    text = text.replace(marker, block, 1)
    path.write_text(text, encoding="utf-8")
