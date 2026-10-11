from pathlib import Path

path = Path('src/dynamics/world.h')
text = path.read_text(encoding='utf-8')
needle = '''    // handle이 가리키는 shape의 Contact / proxy / body list 연결을 정리함.\n    // parent chain의 child segment는 DestroyChain으로만 제거함.\n    void DestroyShape( shapeId shapeId );\n'''
replacement = needle + '''\n    // parent가 없는 chainSegment2의 geometry와 ghost vertex를 같은 shape handle로 교체함.\n    // parent chain child는 adjacency를 parent가 소유하므로 개별 수정하지 않음.\n    void SetChainSegment( shapeId shapeId, const chainSegment2& chainSegment );\n'''
if 'void SetChainSegment( shapeId shapeId, const chainSegment2& chainSegment );' not in text:
    if needle not in text:
        raise SystemExit('shape lifecycle anchor not found')
    text = text.replace(needle, replacement, 1)
    path.write_text(text, encoding='utf-8')
