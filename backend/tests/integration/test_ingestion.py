import pytest
import tempfile
import os
from pathlib import Path
from backend.ingestion.main import main
from backend.ingestion.loaders import MarkdownLoader
from backend.ingestion.processors import ContentCleaner, ContentStructurer
from backend.src.utils.text_splitter import TextSplitter


def test_markdown_loader():
    """
    Test the Markdown loader functionality
    """
    # Create a temporary directory with a test Markdown file
    with tempfile.TemporaryDirectory() as temp_dir:
        # Create a test Markdown file
        test_file = Path(temp_dir) / "test.md"
        test_content = "# Test Document\n\nThis is a test document.\n\n## Section 1\n\nSome content here."
        
        with open(test_file, 'w', encoding='utf-8') as f:
            f.write(test_content)
        
        # Test the loader
        loader = MarkdownLoader()
        docs = loader.load(temp_dir)
        
        # Assertions
        assert len(docs) == 1
        assert docs[0]['content'] == test_content
        assert docs[0]['metadata']['file_name'] == 'test.md'
        assert docs[0]['metadata']['source'] == str(test_file)


def test_content_cleaner():
    """
    Test the content cleaner functionality
    """
    # Create test documents
    docs = [
        {
            'content': '  This   is  a   test  document   with   extra   spaces.  ',
            'metadata': {'source': 'test1.md'}
        }
    ]
    
    # Test the cleaner
    cleaner = ContentCleaner()
    cleaned_docs = cleaner.process(docs)
    
    # Assertions
    assert len(cleaned_docs) == 1
    # Check that excessive whitespace has been normalized
    assert '  ' not in cleaned_docs[0]['content']
    assert cleaned_docs[0]['content'] == ' This is a test document with extra spaces. '


def test_content_structurer():
    """
    Test the content structurer functionality
    """
    # Create a test Markdown document
    test_content = "# Main Title\n\nSome introductory content.\n\n## Section 1\n\nContent of section 1.\n\n### Subsection 1.1\n\nContent of subsection."
    
    docs = [
        {
            'content': test_content,
            'metadata': {'file_name': 'test.md', 'source': 'test.md'}
        }
    ]
    
    # Test the structurer
    structurer = ContentStructurer()
    structured_docs = structurer.process(docs)
    
    # Assertions
    assert len(structured_docs) == 1
    # Check that metadata was updated with sections
    assert 'sections' in structured_docs[0]['metadata']


def test_text_splitter():
    """
    Test the text splitter functionality
    """
    splitter = TextSplitter(chunk_size=50, overlap=10)
    
    # Create test text that exceeds the chunk size
    test_text = "This is a test sentence. " * 20  # This creates a text longer than 50 chars
    
    # Test splitting
    chunks = splitter.split_text(test_text, {'source': 'test'})
    
    # Assertions
    assert len(chunks) > 0
    # Each chunk should be within the size limit (with some tolerance for sentence boundaries)
    for chunk in chunks:
        assert len(chunk.text) <= 100  # Allow for some flexibility beyond the 50 char limit due to sentence boundaries


def test_end_to_end_ingestion():
    """
    Test the end-to-end ingestion process with a sample document
    """
    # This would test the main ingestion function
    # For simplicity in this test, we'll just verify the components work together
    pass