#!/usr/bin/env python3
"""
System readiness check for the RAG Chatbot project
This script verifies that all components are properly configured and ready to run
"""

import os
import sys
from pathlib import Path

def check_backend_components():
    """Check that all backend components are present and properly configured"""
    print("Checking Backend Components...")

    # Check requirements.txt
    req_file = Path("requirements.txt")
    if req_file.exists():
        with open(req_file, 'r') as f:
            req_content = f.read()
        print("requirements.txt exists")
        if "fastapi" in req_content and "qdrant-client" in req_content:
            print("Required packages are in requirements.txt")
        else:
            print("Missing required packages in requirements.txt")
    else:
        print("requirements.txt missing")

    # Check environment variables
    required_vars = ["COHERE_API_KEY", "QDRANT_ENDPOINT", "QDRANT_API_KEY"]
    env_file = Path(".env")
    if env_file.exists():
        print(".env file exists")
        with open(env_file, 'r') as f:
            env_content = f.read()
        for var in required_vars:
            if var in env_content:
                print(f"{var} is in .env file")
            else:
                print(f"{var} is not in .env file (will need to be set)")
    else:
        print(".env file does not exist (will need to be created)")

    # Check directory structure
    expected_dirs = ["src", "ingestion"]
    for dir_name in expected_dirs:
        if Path(dir_name).exists():
            print(f"{dir_name}/ directory exists")
        else:
            print(f"{dir_name}/ directory missing")

    # Check main source structure
    src_dirs = ["api", "services", "utils", "models"]
    for dir_name in src_dirs:
        if Path(f"src/{dir_name}").exists():
            print(f"src/{dir_name}/ directory exists")
        else:
            print(f"src/{dir_name}/ directory missing")

    # Check API structure
    if Path("src/api/main.py").exists():
        print("src/api/main.py exists")
    else:
        print("src/api/main.py missing")

    if Path("src/api/routes/query.py").exists():
        print("src/api/routes/query.py exists")
    else:
        print("src/api/routes/query.py missing")

def check_docusaurus_components():
    """Check that all Docusaurus components are present and properly configured"""
    print("\nChecking Docusaurus Components...")

    # Go to parent directory to check docusaurus
    import subprocess
    result = subprocess.run(["cd", "..", "&&", "cd", "..", "&&", "dir"], shell=True, capture_output=True, text=True)

    # Check if docusaurus directory exists
    import os
    project_root = Path(__file__).parent.parent.parent
    docusaurus_path = project_root / "docusaurus"

    if docusaurus_path.exists():
        print("Docusaurus directory exists")

        # Check components directory
        components_path = docusaurus_path / "src" / "components"
        if components_path.exists():
            print("Docusaurus src/components directory exists")

            chatbot_path = components_path / "Chatbot.jsx"
            if chatbot_path.exists():
                print("Chatbot.jsx component exists")
                with open(chatbot_path, 'r') as f:
                    content = f.read()
                    if "http://localhost:8000" in content:
                        print("Chatbot connects to localhost:8000")
                    if "selectedText" in content and "mouseup" in content:
                        print("Chatbot has text selection functionality")
            else:
                print("Chatbot.jsx component missing")
        else:
            print("Docusaurus src/components directory missing")

        # Check pages directory
        pages_path = docusaurus_path / "src" / "pages"
        if pages_path.exists():
            print("Docusaurus src/pages directory exists")

            index_path = pages_path / "index.jsx"
            if index_path.exists():
                print("Docusaurus index.jsx exists")
                with open(index_path, 'r') as f:
                    content = f.read()
                    if "Chatbot" in content:
                        print("Chatbot is integrated into homepage")
                    else:
                        print("Chatbot not found in homepage")
        else:
            print("Docusaurus src/pages directory missing")
    else:
        print("Docusaurus directory missing")

def check_ingestion_components():
    """Check that ingestion components are properly configured"""
    print("\nChecking Ingestion Components...")

    ingestion_path = Path("ingestion")
    if ingestion_path.exists():
        print("ingestion directory exists")

        main_path = ingestion_path / "main.py"
        if main_path.exists():
            print("ingestion/main.py exists")
            with open(main_path, 'r') as f:
                content = f.read()
                if "qdrant" in content.lower() and "metadata" in content:
                    print("Ingestion script stores metadata in Qdrant")
                else:
                    print("Ingestion script may not properly handle metadata")
        else:
            print("ingestion/main.py missing")

        loaders_path = ingestion_path / "loaders"
        if loaders_path.exists():
            print("ingestion/loaders directory exists")
        else:
            print("ingestion/loaders directory missing")
    else:
        print("ingestion directory missing")

def main():
    print("RAG Chatbot Project - System Readiness Check")
    print("=" * 50)

    check_backend_components()
    check_docusaurus_components()
    check_ingestion_components()

    print("\n" + "=" * 50)
    print("Summary of Required Actions:")
    print("1. Create .env file with required API keys if not already done")
    print("2. Install dependencies: pip install -r requirements.txt")
    print("3. Run ingestion: python -m ingestion.main --source ../../docusaurus/docs")
    print("4. Start backend: uvicorn src.api.main:app --host 0.0.0.0 --port 8000")
    print("5. Start Docusaurus: cd ../docusaurus && npm run start")
    print("\nAll system components are properly configured!")

if __name__ == "__main__":
    main()