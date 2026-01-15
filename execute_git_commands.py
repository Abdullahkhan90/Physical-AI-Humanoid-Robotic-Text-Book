#!/usr/bin/env python3
import subprocess
import sys

def run_cmd(cmd):
    print(f"Running: {cmd}")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    print(f"Exit code: {result.returncode}")
    if result.stdout:
        print(f"Stdout: {result.stdout}")
    if result.stderr:
        print(f"Stderr: {result.stderr}")
    return result.returncode == 0

# Configure git
success = True
success &= run_cmd('git config --global user.email "user@example.com"')
success &= run_cmd('git config --global user.name "User"')

# Add files
success &= run_cmd('git add backend/src/services/qdrant_service.py backend/ingestion/ingest.py')

# Commit
success &= run_cmd('git commit -m "Switched Qdrant to cloud with correct URL and API key - fixed 404 error"')

# Push
success &= run_cmd('git push origin main')

if success:
    print("All git operations completed successfully!")
else:
    print("Some git operations failed.")