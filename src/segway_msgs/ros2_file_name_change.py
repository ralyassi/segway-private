import os
import re

def to_pascal_case(s):
    # Remove extension, split by underscores, capitalize each part, join
    base, ext = os.path.splitext(s)
    if ext not in ['.msg', '.srv', '.action']:
        return None
    parts = base.split('_')
    return ''.join(word.capitalize() for word in parts) + ext

folders = ['msg', 'srv', 'action']

for folder in folders:
    if not os.path.exists(folder):
        continue
    for filename in os.listdir(folder):
        if filename.endswith(('.msg', '.srv', '.action')):
            new_name = to_pascal_case(filename)
            if new_name and new_name != filename:
                src = os.path.join(folder, filename)
                dst = os.path.join(folder, new_name)
                print(f"Renaming: {src} -> {dst}")
                os.rename(src, dst)

