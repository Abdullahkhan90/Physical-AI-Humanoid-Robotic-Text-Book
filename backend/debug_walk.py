import os

# Check the docs directory with absolute path
base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
docs_path = os.path.join(base_path, 'docusaurus', 'docs')
print(f"Base path: {base_path}")
print(f"Docs path: {docs_path}")
print(f"Path exists: {os.path.exists(docs_path)}")
print(f"Is directory: {os.path.isdir(docs_path)}")

# List all contents
all_contents = []
for root, dirs, files in os.walk(docs_path):
    print(f"Root: {root}")
    print(f"Dirs: {dirs}")
    print(f"Files: {files}")
    all_contents.extend([os.path.join(root, f) for f in files if f.endswith('.md')])

print(f"\nAll markdown files found: {all_contents}")
print(f"Total markdown files: {len(all_contents)}")