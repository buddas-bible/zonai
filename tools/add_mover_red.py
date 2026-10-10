from pathlib import Path

path = Path("tests/sandbox/joint_demo_ui_test.cpp")
text = path.read_text(encoding="utf-8")
marker = '''    if( weldVertices <= 0 ) fail( "Weld joint view drew no anchor or frame overlay" );

    ImGui::DestroyContext();
'''
block = '''    if( weldVertices <= 0 ) fail( "Weld joint view drew no anchor or frame overlay" );

    // Mover view는 base scene 위에 target velocity / reaction force overlay를 추가해야 함.
    auto moverBase = createRigidBodyDemo( demoKind::moverJointPlayground );
    auto moverView = createJointDemoView( demoKind::moverJointPlayground );
    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
    ImGui::SetNextWindowSize( { 420.0f, 1400.0f } );
    ImGui::Begin( "Mover inspector" );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "JointQuickSettings" ), 1 );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "MoverJointInspector" ), 1 );
    moverView->drawControls();

    debugCamera moverCamera{};
    moverCamera.center = getDemoEntry( demoKind::moverJointPlayground ).cameraCenter;
    moverCamera.pixelsPerMeter = getDemoEntry( demoKind::moverJointPlayground ).pixelsPerMeter;
    ImDrawList* moverList = ImGui::GetWindowDrawList();

    const int moverBaseBefore = moverList->VtxBuffer.Size;
    debugDraw moverBaseDraw{ moverList, moverCamera, { 0.0f, 0.0f }, { 800.0f, 600.0f } };
    moverBase->draw( moverBaseDraw );
    const int moverBaseVertices = moverList->VtxBuffer.Size - moverBaseBefore;

    const int moverViewBefore = moverList->VtxBuffer.Size;
    debugDraw moverViewDraw{ moverList, moverCamera, { 0.0f, 0.0f }, { 800.0f, 600.0f } };
    moverView->draw( moverViewDraw );
    const int moverViewVertices = moverList->VtxBuffer.Size - moverViewBefore;

    ImGui::End();
    ImGui::Render();

    if( moverViewVertices <= moverBaseVertices ) fail( "Mover joint view drew no velocity/reaction overlay" );

    ImGui::DestroyContext();
'''
if block not in text:
    if marker not in text:
        raise RuntimeError("Mover UI RED marker not found")
    text = text.replace(marker, block, 1)
    path.write_text(text, encoding="utf-8")
