#!/usr/bin/env python3
"""Fix Rust warnings systematically by reading cargo output and patching files."""

import re
import subprocess
import os
import sys

ENV = os.environ.copy()
ENV['PATH'] = os.path.expanduser('~/.cargo/bin') + ':' + ENV['PATH']

def run_cargo_check():
    """Run cargo check and return list of (file, line, col, message) tuples."""
    result = subprocess.run(
        ['cargo', 'check', '--release'],
        capture_output=True, text=True, env=ENV, cwd='/home/z/my-project'
    )
    out = result.stdout + result.stderr
    pattern = re.compile(r'^warning: (.*?)\n\s+--> (.*?):(\d+):(\d+)', re.MULTILINE)
    warnings = []
    for m in pattern.finditer(out):
        msg = m.group(1)
        file = m.group(2)
        line = int(m.group(3))
        col = int(m.group(4))
        warnings.append((file, line, col, msg))
    return warnings

def read_lines(path):
    with open(path) as f:
        return f.readlines()

def write_lines(path, lines):
    with open(path, 'w') as f:
        f.writelines(lines)

def fix_unused_variable(lines, line_no, col, var_name):
    """Prefix unused variable with _ on its declaration line."""
    idx = line_no - 1
    line = lines[idx]
    # Find the variable name on this line at or after col
    # Pattern: let mut? NAME  or  let NAME =  or  for NAME in
    # We need to be careful to only prefix the actual variable binding
    # Common patterns:
    #   let NAME =
    #   let mut NAME =
    #   for NAME in
    #   NAME: Type,
    # We'll try each in turn
    # Try `let mut NAME` or `let NAME`
    patterns = [
        (rf'\blet\s+mut\s+{re.escape(var_name)}\b', f'let mut _{var_name}'),
        (rf'\blet\s+{re.escape(var_name)}\b', f'let _{var_name}'),
        (rf'\bfor\s+{re.escape(var_name)}\s+in\b', f'for _{var_name} in'),
        (rf'\b{re.escape(var_name)}\s*:', f'_{var_name}:'),  # struct field or fn param
    ]
    new_line = line
    for pat, rep in patterns:
        new_line, n = re.subn(pat, rep, new_line, count=1)
        if n > 0:
            break
    if new_line != line:
        lines[idx] = new_line
        return True
    return False

def fix_unused_import(lines, line_no, col, items_str):
    """Remove specific items from a use statement on the line."""
    idx = line_no - 1
    line = lines[idx]
    # Items come as `Foo`, `Foo and Bar`, or `Foo, Bar, Baz`
    # Extract item names
    items = re.findall(r'`([^`]+)`', items_str)
    if not items:
        return False
    # Multiple cases:
    # use foo::{A, B, C};
    # use foo::A;
    # use foo::A as B;
    for item in items:
        # Remove the item from the use statement
        # Pattern: `{A, B, C}` -> remove A
        # Pattern: `A` (single) -> remove whole use
        # Pattern: `A, B` -> remove A
        # First try in braces
        # e.g. {A, B, C} - remove A
        pat1 = rf'{{{re.escape(item)}\s*,\s*'  # {A, 
        pat2 = rf',\s*{re.escape(item)}\s*}}'   # , A}
        pat3 = rf'{{{re.escape(item)}\s*}}}}'    # {A}
        pat4 = rf'{{{re.escape(item)}\s*,\s*}}}}' # {A,}
        new_line = line
        # Remove `item,` or `, item` from inside braces
        new_line = re.sub(rf'{{{re.escape(item)}\s*,\s*', '{', new_line)
        new_line = re.sub(rf',\s*{re.escape(item)}\s*}}', '}', new_line)
        # If only item remains in braces, remove the whole use
        new_line = re.sub(rf'::\s*{{{re.escape(item)}\s*}}}}', '', new_line)
        line = new_line
    # If we end up with `use ;` or `use foo::;` remove the whole line
    if re.match(r'\s*use\s*;\s*', line) or re.match(r'\s*use\s+\w+::;\s*', line):
        lines[idx] = ''  # remove line
    else:
        lines[idx] = line
    return True

def fix_non_snake_case(lines, line_no, col, old_name, new_name):
    """Rename a variable from non_snake_case to snake_case."""
    idx = line_no - 1
    line = lines[idx]
    # Replace only the binding declaration, not uses
    # Common: `let OLD =` -> `let NEW =`
    new_line = re.sub(rf'\blet\s+{re.escape(old_name)}\b', f'let {new_name}', line, count=1)
    if new_line != line:
        lines[idx] = new_line
        # Now rename all uses within the function scope (heuristic: until next `let` or `}` at same indent)
        # For simplicity, replace within the next 100 lines
        for i in range(idx + 1, min(idx + 100, len(lines))):
            cur = lines[i]
            # Stop at function boundary
            if re.match(r'^\}\s*$', cur):
                break
            new_cur = re.sub(rf'\b{re.escape(old_name)}\b', new_name, cur)
            lines[i] = new_cur
        return True
    return False

def fix_unreachable_pattern(lines, line_no, col):
    """Add #[allow(unreachable_patterns)] above the line."""
    idx = line_no - 1
    # Get the indentation of the match arm
    line = lines[idx]
    # The unreachable pattern is usually the `_ => ...` arm
    # We can't easily add the attribute to a single arm, so add it to the match's parent
    # Actually, easier: add `#[allow(unreachable_patterns)]` as a crate-level allow via lib.rs
    pass  # handle separately

def main():
    warnings = run_cargo_check()
    print(f"Found {len(warnings)} warnings")
    
    # Group by file for batching
    from collections import defaultdict
    by_file = defaultdict(list)
    for w in warnings:
        by_file[w[0]].append(w)
    
    fixed = 0
    for file, ws in by_file.items():
        if not os.path.exists(file):
            continue
        lines = read_lines(file)
        original = list(lines)
        # Sort by line desc so we can patch without affecting later line numbers
        ws_sorted = sorted(ws, key=lambda w: -w[1])
        
        for file_path, line, col, msg in ws_sorted:
            # Match the warning type
            m = re.match(r'unused variable: `([^`]+)`', msg)
            if m:
                var = m.group(1)
                if fix_unused_variable(lines, line, col, var):
                    fixed += 1
                continue
            
            m = re.match(r'unused imports?: (.*)', msg)
            if m:
                items_str = m.group(1)
                if fix_unused_import(lines, line, col, items_str):
                    fixed += 1
                continue
            
            m = re.match(r"variable `(\w+)` should have a snake case name", msg)
            if m:
                old_name = m.group(1)
                # Suggested new name from message (we just lowercase letters after first)
                new_name = re.sub(r'([A-Z])', lambda x: '_' + x.group(1).lower(), old_name).lstrip('_')
                # Actually the compiler suggests: e.g. R -> r, pN -> p_n, expected_K -> expected_k
                # Simple: lowercase all uppercase letters, prefix with _ if mid-identifier
                # Use the standard snake_case conversion
                new_name = re.sub(r'(?<!^)(?=[A-Z])', '_', old_name).lower()
                if fix_non_snake_case(lines, line, col, old_name, new_name):
                    fixed += 1
                continue
            
            m = re.match(r"variable `(\w+)` is assigned to, but never used", msg)
            if m:
                var = m.group(1)
                # Find the let binding and prefix with _
                if fix_unused_variable(lines, line, col, var):
                    fixed += 1
                continue
            
            m = re.match(r"value assigned to `(\w+)` is never read", msg)
            if m:
                var = m.group(1)
                # The line where the assignment happens - prefix the variable
                # Heuristic: find `var = ...` on this line and change to `_var = ...`
                # Actually we should find the original binding, but that's elsewhere
                # For now, comment out the assignment or change to `_`
                idx = line - 1
                cur_line = lines[idx]
                # Replace `var = expr;` with `let _ = expr;` or just remove the assignment
                # Simplest: replace `var =` with `let _ =`
                new_line = re.sub(rf'\b{re.escape(var)}\s*=', 'let _ =', cur_line, count=1)
                if new_line != cur_line:
                    lines[idx] = new_line
                    fixed += 1
                continue
            
            m = re.match(r"variable does not need to be mutable", msg)
            if m:
                idx = line - 1
                cur_line = lines[idx]
                # Remove `mut` from this line
                new_line = re.sub(r'\blet\s+mut\s+', 'let ', cur_line, count=1)
                if new_line != cur_line:
                    lines[idx] = new_line
                    fixed += 1
                continue
        
        if lines != original:
            write_lines(file, lines)
    
    print(f"Fixed {fixed} warnings")

if __name__ == '__main__':
    main()
