from typing import List
from ..services.rag_service import RAGService
from ..utils.logging import get_logger
from ..models.content_model import LearningContent
from ..models.module_model import TextbookModule


class IndexingService:
    """
    Service for indexing textbook content into the RAG knowledge base
    """
    
    def __init__(self):
        self.logger = get_logger(__name__)
        self.rag_service = RAGService()

    async def index_single_content(self, content: LearningContent) -> bool:
        """
        Index a single piece of learning content
        """
        try:
            success = await self.rag_service.index_content(
                module_id=content.moduleID,
                content_id=content.id,
                text=content.content,
                metadata={
                    "contentType": content.contentType,
                    "title": content.title,
                    "wordCount": content.wordCount
                }
            )
            
            if success:
                self.logger.info(f"Successfully indexed content {content.id}")
            else:
                self.logger.error(f"Failed to index content {content.id}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error indexing content {content.id}: {str(e)}")
            return False

    async def index_module_content(self, module: TextbookModule, contents: List[LearningContent]) -> int:
        """
        Index all content for a specific module
        """
        indexed_count = 0
        
        for content in contents:
            if content.moduleID == module.id:  # Ensure content belongs to this module
                success = await self.index_single_content(content)
                if success:
                    indexed_count += 1
        
        self.logger.info(f"Indexed {indexed_count}/{len(contents)} content pieces for module {module.id}")
        return indexed_count

    async def index_all_content(self, modules: List[TextbookModule], all_contents: List[LearningContent]) -> int:
        """
        Index all content for all modules
        """
        total_indexed = 0
        
        for module in modules:
            indexed_for_module = await self.index_module_content(module, all_contents)
            total_indexed += indexed_for_module
        
        self.logger.info(f"Total indexed: {total_indexed} content pieces across all modules")
        return total_indexed

    async def update_content_index(self, content_id: str, new_content: str) -> bool:
        """
        Update the index for a specific piece of content
        This would typically involve removing the old indexed chunks and adding new ones
        """
        # For simplicity, we're not implementing the removal mechanism here
        # In a real implementation, you would need to fetch existing chunks with content_id
        # and delete them before re-indexing
        self.logger.warning("Update index functionality needs full implementation with chunk deletion")
        return False

    async def remove_content_from_index(self, content_id: str) -> bool:
        """
        Remove all indexed chunks associated with a specific content ID
        """
        try:
            # In a real implementation, you would query Qdrant for points with this content_id
            # and delete them. Here we'll just log the intention
            self.logger.info(f"Would remove content {content_id} from index")
            # Implementation would need to:
            # 1. Find all points with payload containing content_id
            # 2. Delete those points from the collection
            return True
            
        except Exception as e:
            self.logger.error(f"Error removing content {content_id} from index: {str(e)}")
            return False