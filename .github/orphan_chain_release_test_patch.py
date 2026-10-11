from pathlib import Path

path = Path('tests/dynamics/body_shape_contact_lifecycle_test.cpp')
text = path.read_text(encoding='utf-8')
old = '    check( after.ghost1 == before.ghost1 && after.segment.a == before.segment.a && after.segment.b == before.segment.b && after.ghost2 == before.ghost2, "parent chain child geometry mutated" );'
new = '    check( after.ghost1.x == before.ghost1.x && after.ghost1.y == before.ghost1.y && after.segment.a.x == before.segment.a.x && after.segment.a.y == before.segment.a.y && after.segment.b.x == before.segment.b.x && after.segment.b.y == before.segment.b.y && after.ghost2.x == before.ghost2.x && after.ghost2.y == before.ghost2.y, "parent chain child geometry mutated" );'
if old not in text:
    raise SystemExit('release no-op assertion anchor not found')
path.write_text(text.replace(old, new, 1), encoding='utf-8')
