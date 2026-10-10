from pathlib import Path
import subprocess

base = "6b75eee9f8f2b8a0a708d952c33ec2c282680813"
path = "tests/sandbox/demo_test.cpp"
text = subprocess.check_output( [ "git", "show", f"{base}:{path}" ], text=True )
old = 'check( getDemoEntries().size() == 10, "ten independently selectable demo entries" );'
new = 'check( getDemoEntries().size() == 11, "eleven independently selectable demo entries" );'
if text.count( old ) != 1:
    raise RuntimeError( "Sandbox demo-count marker missing from stacked base" )
Path( path ).write_text( text.replace( old, new, 1 ), encoding="utf-8" )
print( "Restored full Sandbox regression test and updated only the demo count" )
