"""
Document loader utilities for the ingestion pipeline
"""

import os
from typing import List, Dict, Any
from pathlib import Path


class DocumentLoader:
    """
    Base class for loading documents from various sources
    """
    def load(self, path: str) -> List[Dict[str, Any]]:
        """
        Load documents from the given path
        Returns a list of dictionaries with 'content' and 'metadata' keys
        """
        raise NotImplementedError


class MarkdownLoader(DocumentLoader):
    """
    Load Markdown files from a directory
    """
    def load(self, path: str) -> List[Dict[str, Any]]:
        """
        Load all Markdown files from the specified directory
        """
        docs = []
        
        # Convert to Path object
        path_obj = Path(path)
        
        # Find all .md files in the directory and subdirectories
        md_files = path_obj.rglob("*.md")
        
        for file_path in md_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                # Create metadata
                metadata = {
                    'source': str(file_path),
                    'file_path': str(file_path),
                    'file_name': file_path.name,
                    'directory': str(file_path.parent),
                }
                
                docs.append({
                    'content': content,
                    'metadata': metadata
                })
            except Exception as e:
                print(f"Error loading file {file_path}: {e}")
                
        return docs


class TextLoader(DocumentLoader):
    """
    Load plain text files from a directory
    """
    def load(self, path: str) -> List[Dict[str, Any]]:
        """
        Load all text files from the specified directory
        """
        docs = []
        
        # Convert to Path object
        path_obj = Path(path)
        
        # Find all .txt files in the directory and subdirectories
        txt_files = path_obj.rglob("*.txt")
        
        for file_path in txt_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                # Create metadata
                metadata = {
                    'source': str(file_path),
                    'file_path': str(file_path),
                    'file_name': file_path.name,
                    'directory': str(file_path.parent),
                }
                
                docs.append({
                    'content': content,
                    'metadata': metadata
                })
            except Exception as e:
                print(f"Error loading file {file_path}: {e}")
                
        return docs