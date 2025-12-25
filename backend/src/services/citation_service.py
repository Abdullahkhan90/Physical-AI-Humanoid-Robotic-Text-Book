from typing import List, Dict
import logging

logger = logging.getLogger(__name__)

class CitationService:
    def __init__(self):
        pass
    
    def format_citations(self, retrieved_contexts: List[Dict]) -> List[Dict]:
        """
        Format retrieved contexts into proper citation format
        """
        citations = []
        for ctx in retrieved_contexts:
            try:
                payload = ctx.get("payload", {})
                # Extract chapter and section from Qdrant payload
                chapter = payload.get("chapter", "Unknown Chapter")
                section = payload.get("section", "Unknown Section")
                file_path = payload.get("file_path", "Unknown File")

                # Create section title combining chapter and section
                if chapter != "Unknown Chapter" and section != "Unknown Section":
                    formatted_section = f"{chapter} - {section}"
                elif chapter != "Unknown Chapter":
                    formatted_section = chapter
                elif section != "Unknown Section":
                    formatted_section = section
                else:
                    formatted_section = "Unknown Section"

                # Create URL from file_path if it exists
                if file_path and file_path != "Unknown File":
                    # Create full URL for the documentation site
                    url = f"https://abdullahkhan90.github.io/Physical-AI-Humanoid-Robotic-Text-Book/{file_path}"
                else:
                    url = "#"  # Default fallback that satisfies validation

                citations.append({
                    "section": formatted_section,
                    "url": url,
                    "text": payload.get("text", "")[:200] + "..." if len(payload.get("text", "")) > 200 else payload.get("text", ""),
                    "similarity_score": ctx.get("score", 0.0)
                })
            except Exception as e:
                logger.warning(f"Error formatting citation for context {ctx.get('id', 'unknown')}: {e}")
                # Add a basic citation even if there's an error
                citations.append({
                    "section": "Unknown Section",
                    "url": "#",  # Default fallback
                    "text": ctx.get("text", "")[:200] + "...",
                    "similarity_score": ctx.get("score", 0.0)
                })

        return citations
    
    def validate_citations(self, citations: List[Dict], required_fields: List[str] = ["section", "url", "text"]) -> bool:
        """
        Validate that citations have required fields
        """
        for citation in citations:
            for field in required_fields:
                if field not in citation or not citation[field]:
                    logger.warning(f"Missing or empty field '{field}' in citation: {citation}")
                    return False
        return True
    
    def get_top_citations(self, citations: List[Dict], top_k: int = 5) -> List[Dict]:
        """
        Get top-k citations based on similarity score
        """
        # Sort citations by similarity score in descending order
        sorted_citations = sorted(citations, key=lambda x: x.get("similarity_score", 0.0), reverse=True)
        return sorted_citations[:top_k]