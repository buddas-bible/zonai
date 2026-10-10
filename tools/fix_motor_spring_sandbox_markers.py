from pathlib import Path

files = {
    Path("sandbox/rigidBodyJointDemo.cpp"): [
        ("#pragma endregion Presets\n#pragma endregion Presets\n", "#pragma endregion Presets\n"),
    ],
    Path("sandbox/jointDemoView.cpp"): [
        ("            else if( kind == demoKind::weldPair )\n            else if( kind == demoKind::weldPair )\n", "            else if( kind == demoKind::weldPair )\n"),
        ("        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n", "        if( view_.getKind() == demoKind::weldPair && view_.getWorld().IsValid( view_.getWeldJoint() ) )\n"),
        ("    void drawWeldInspector()\n    void drawWeldInspector()\n", "    void drawWeldInspector()\n"),
    ],
}

for path, replacements in files.items():
    text = path.read_text(encoding="utf-8")
    for old, new in replacements:
        if text.count(old) != 1:
            raise SystemExit(f"{path}: expected one duplicate marker for {old!r}")
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
