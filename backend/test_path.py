import os

# This mimics how the path is resolved in the ingest.py script
file_path = os.path.abspath(__file__)  # This file's path
print(f"Current file path: {file_path}")

parent_dir = os.path.dirname(file_path)  # ingestion directory
print(f"Parent directory: {parent_dir}")

parent_parent_dir = os.path.dirname(parent_dir)  # backend directory
print(f"Parent parent directory: {parent_parent_dir}")

parent_parent_parent_dir = os.path.dirname(parent_parent_dir)  # Project root directory
print(f"Project root: {parent_parent_parent_dir}")

docs_path = os.path.join(parent_parent_parent_dir, "docusaurus", "docs")
print(f"Docs path: {docs_path}")
print(f"Does docs path exist? {os.path.exists(docs_path)}")

# Count markdown files
if os.path.exists(docs_path):
    md_files = []
    for root, dirs, files in os.walk(docs_path):
        md_files.extend([f for f in files if f.endswith('.md')])
    print(f"Found {len(md_files)} markdown files: {md_files}")
else:
    print("Docs path does not exist!")