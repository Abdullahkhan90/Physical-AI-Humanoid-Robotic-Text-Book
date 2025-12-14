from typing import List, Dict, Optional
from ..models.citation_model import Citation
import requests
import logging


class CitationService:
    """
    Service for managing citations according to APA format
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
    
    def validate_citation(self, citation: Citation) -> bool:
        """
        Validate a citation according to the project's requirements
        """
        # Check if required fields are present
        if not citation.title or not citation.title.strip():
            self.logger.error("Citation missing title")
            return False
            
        if not citation.authors or len(citation.authors) == 0:
            self.logger.error("Citation missing authors")
            return False
            
        if not citation.source or not citation.source.strip():
            self.logger.error("Citation missing source")
            return False
            
        if citation.year < 1900 or citation.year > 2030:
            self.logger.error(f"Invalid year in citation: {citation.year}")
            return False
            
        if not citation.citationType or citation.citationType not in ["journalArticle", "conferencePaper", "book", "online"]:
            self.logger.error(f"Invalid citation type: {citation.citationType}")
            return False
            
        if not citation.apaFormatted or not citation.apaFormatted.strip():
            self.logger.error("Citation missing APA formatted string")
            return False
            
        return True
    
    def format_citation_apa(self, title: str, authors: List[str], source: str, year: int, 
                           doi: Optional[str] = None, url: Optional[str] = None, 
                           citation_type: Optional[str] = "journalArticle") -> str:
        """
        Format a citation in APA format
        """
        # Basic APA format: Author, A. A. (Year). Title. Source.
        author_str = self._format_authors(authors)
        result = f"{author_str} ({year}). {title}. {source}."
        
        if doi:
            result += f" https://doi.org/{doi}"
        elif url:
            result += f" {url}"
            
        return result
    
    def _format_authors(self, authors: List[str]) -> str:
        """
        Format author names according to APA style
        """
        if len(authors) == 0:
            return ""
        
        if len(authors) == 1:
            return authors[0]
        elif len(authors) == 2:
            return f"{authors[0]} & {authors[1]}"
        else:
            # For 3+ authors, list first author followed by et al.
            return f"{authors[0]} et al."
    
    def check_citation_quality(self, citation: Citation) -> Dict[str, bool]:
        """
        Check if citation meets quality standards (e.g., peer-reviewed)
        """
        quality_report = {
            'has_doi': citation.doi is not None,
            'is_academic_source': self._is_academic_source(citation.source),
            'is_recent': citation.year >= 2010,  # Considered recent in tech field
            'has_valid_authors': len(citation.authors) > 0
        }
        
        return quality_report
    
    def _is_academic_source(self, source: str) -> bool:
        """
        Check if the source is likely to be academic (journal, conference, etc.)
        """
        academic_indicators = [
            'journal', 'conference', 'proceedings', 'transactions', 
            'academic', 'research', 'science', 'ieee', 'acm', 'arxiv'
        ]
        
        source_lower = source.lower()
        for indicator in academic_indicators:
            if indicator in source_lower:
                return True
                
        return False
    
    def verify_citation_access(self, citation: Citation) -> bool:
        """
        Verify that the citation can be accessed (if it has a URL)
        """
        if not citation.url:
            return True  # If no URL, we can't verify access, so assume OK
            
        try:
            response = requests.head(citation.url, timeout=10)
            return response.status_code < 400
        except requests.RequestException:
            self.logger.warning(f"Could not verify access to citation URL: {citation.url}")
            return False