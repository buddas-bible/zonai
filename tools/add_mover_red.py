from pathlib import Path

path = Path("tests/sandbox/joint_demo_ui_test.cpp")
text = path.read_text(encoding="utf-8")
marker = '''    if( weldVertices <= 0 ) fail( "Weld joint view drew no anchor or frame overlay" );

    ImGui::DestroyContext();
'''
block = '''    if( weldVertices <= 0 ) fail( "Weld joint view drew no anchor or frame overlay" );

    // Mover view는 quick presets / inspector를 실제 ImGui controls로 노출해야 함.
    // 현재 구현 전에는 drawControls()가 Mover kind를 early-return하므로 cursor가 전혀 진행되지 않음.
    auto moverView = createJointDemoView( demoKind::moverJointPlayground );
    ImGui::NewFrame();
    ImGui::SetNextWindowPos( { 0.0f, 0.0f } );
    ImGui::SetNextWindowSize( { 420.0f, 1400.0f } );
    ImGui::Begin( "Mover inspector" );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "JointQuickSettings" ), 1 );
    ImGui::GetStateStorage()->SetInt( ImGui::GetID( "MoverJointInspector" ), 1 );
    const float moverCursorBefore = ImGui::GetCursorPosY();
    moverView->drawControls();
    const float moverCursorAfter = ImGui::GetCursorPosY();
    ImGui::End();
    ImGui::Render();

    if( moverCursorAfter <= moverCursorBefore ) fail( "Mover joint view drew no controls" );

    ImGui::DestroyContext();
'''
if block not in text:
    if marker not in text:
        raise RuntimeError("Mover UI RED marker not found")
    text = text.replace(marker, block, 1)
    path.write_text(text, encoding="utf-8")
