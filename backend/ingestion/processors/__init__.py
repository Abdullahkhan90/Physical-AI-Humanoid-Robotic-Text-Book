"""
Text processing utilities for the ingestion pipeline
"""

import re
from typing import List, Dict, Any
from src.utils.text_splitter import TextSplitter, TextChunk


class TextProcessor:
    """
    Base class for processing text content
    """
    def process(self, documents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Process the list of documents
        """
        raise NotImplementedError


class ContentCleaner(TextProcessor):
    """
    Clean and normalize content before chunking
    """
    def process(self, documents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Clean content in the documents
        """
        cleaned_docs = []
        
        for doc in documents:
            content = doc['content']
            
            # Remove excessive whitespace
            content = re.sub(r'\s+', ' ', content)
            
            # Remove special characters if needed
            # content = re.sub(r'[^\w\s]', ' ', content)
            
            cleaned_doc = {
                'content': content,
                'metadata': doc['metadata']
            }
            
            cleaned_docs.append(cleaned_doc)
        
        return cleaned_docs


class ContentStructurer(TextProcessor):
    """
    Structure content based on document type (e.g., Markdown)
    """
    def process(self, documents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Extract structure from content (headings, sections, etc.)
        """
        structured_docs = []
        
        for doc in documents:
            content = doc['content']
            metadata = doc['metadata']
            
            # For Markdown files, try to extract sections
            if metadata.get('file_name', '').endswith('.md'):
                sections = self._extract_markdown_sections(content)
                
                # Add section information to metadata
                if sections:
                    metadata['sections'] = sections
            
            structured_doc = {
                'content': content,
                'metadata': metadata
            }
            
            structured_docs.append(structured_doc)
        
        return structured_docs
    
    def _extract_markdown_sections(self, content: str) -> List[Dict[str, str]]:
        """
        Extract sections from Markdown content
        """
        sections = []
        
        # Pattern to match Markdown headings
        heading_pattern = r'^(#{1,6})\s+(.+)$'
        lines = content.split('\n')
        
        current_section = ""
        current_level = 0
        
        for line in lines:
            match = re.match(heading_pattern, line.strip())
            if match:
                # Save previous section if exists
                if current_section.strip():
                    sections.append({
                        'level': current_level,
                        'title': current_section,
                        'content': ''  # Actual content would be extracted in a full implementation
                    })
                
                # Start new section
                hashes, title = match.groups()
                current_level = len(hashes)
                current_section = title.strip()
            else:
                # Accumulate content for the current section
                pass
        
        # Add the last section
        if current_section.strip():
            sections.append({
                'level': current_level,
                'title': current_section,
                'content': ''  # Actual content would be extracted in a full implementation
            })
        
        return sections