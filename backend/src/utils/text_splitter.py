import re
from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class TextChunk:
    text: str
    metadata: dict

class TextSplitter:
    """
    Intelligent text splitter that preserves document hierarchy
    """
    
    def __init__(self, chunk_size: int = 800, overlap: int = 100):
        self.chunk_size = chunk_size
        self.overlap = overlap

    def split_text(self, text: str, metadata: dict = None) -> List[TextChunk]:
        """
        Split text into chunks while preserving document structure
        """
        if metadata is None:
            metadata = {}
        
        # Split by paragraphs first
        paragraphs = text.split('\n\n')
        
        chunks = []
        current_chunk = ""
        current_metadata = metadata.copy()
        
        for paragraph in paragraphs:
            # If adding this paragraph would exceed chunk size,
            # save the current chunk and start a new one
            if len(current_chunk) + len(paragraph) > self.chunk_size and current_chunk:
                chunks.append(TextChunk(text=current_chunk.strip(), metadata=current_metadata))
                # Add overlap from the end of the previous chunk
                overlap_start = max(0, len(current_chunk) - self.overlap)
                current_chunk = current_chunk[overlap_start:] + paragraph
            else:
                current_chunk += "\n\n" + paragraph
        
        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append(TextChunk(text=current_chunk.strip(), metadata=metadata))
        
        # If any chunk is still too large, split it by sentences
        final_chunks = []
        for chunk in chunks:
            if len(chunk.text) > self.chunk_size:
                final_chunks.extend(self._split_large_chunk(chunk))
            else:
                final_chunks.append(chunk)
        
        return final_chunks

    def _split_large_chunk(self, chunk: TextChunk) -> List[TextChunk]:
        """
        Split a large chunk by sentences
        """
        # Split by sentences
        sentences = re.split(r'[.!?]+', chunk.text)
        
        chunks = []
        current_chunk_text = ""
        
        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue
                
            # Add sentence to current chunk if it doesn't exceed size
            if len(current_chunk_text) + len(sentence) <= self.chunk_size:
                current_chunk_text += sentence + ". "
            else:
                # If current chunk has content, save it and start a new one
                if current_chunk_text.strip():
                    chunks.append(TextChunk(text=current_chunk_text.strip(), metadata=chunk.metadata))
                
                # If the sentence itself is too long, we need to break it
                if len(sentence) > self.chunk_size:
                    # Break very long sentence into smaller parts
                    sub_chunks = self._break_very_long_sentence(sentence)
                    for sub_chunk in sub_chunks:
                        chunks.append(TextChunk(text=sub_chunk, metadata=chunk.metadata))
                    current_chunk_text = ""
                else:
                    current_chunk_text = sentence + ". "
        
        # Add the last chunk if it has content
        if current_chunk_text.strip():
            chunks.append(TextChunk(text=current_chunk_text.strip(), metadata=chunk.metadata))
        
        return chunks

    def _break_very_long_sentence(self, sentence: str) -> List[str]:
        """
        Break a very long sentence into smaller parts
        """
        words = sentence.split()
        chunks = []
        current_chunk = ""
        
        for word in words:
            if len(current_chunk + " " + word) <= self.chunk_size:
                current_chunk += " " + word
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                current_chunk = word
        
        if current_chunk.strip():
            chunks.append(current_chunk.strip())
        
        return chunks