from fastapi import APIRouter, HTTPException, Query
from typing import List
from pydantic import ValidationError
from ..models.content_model import LearningContent
from ..utils.logging import get_logger

router = APIRouter(prefix="/content", tags=["content"])
logger = get_logger(__name__)

# Mock data - in real implementation this would come from a database or file system
content_data: List[LearningContent] = [
    LearningContent(
        id="content-1-1",
        title="Introduction to ROS 2",
        content="# Introduction to ROS 2\n\nROS 2 (Robot Operating System 2) is a flexible framework for writing robot applications...",
        moduleID="module-1-ros2",
        contentType="theory",
        wordCount=1200,
        estimatedReadingTime=5,
        requiredCitations=[]
    ),
    LearningContent(
        id="content-1-2",
        title="ROS 2 Nodes, Topics, and Services",
        content="# ROS 2 Nodes, Topics, and Services\n\nIn ROS 2, nodes communicate through a publish-subscribe model using topics, services, and actions...",
        moduleID="module-1-ros2",
        contentType="theory",
        wordCount=1500,
        estimatedReadingTime=6,
        requiredCitations=[]
    )
]


@router.get("/{content_id}", response_model=LearningContent)
async def get_content_by_id(content_id: str):
    """
    Retrieve a specific content piece by its ID
    """
    try:
        # Validate content_id format
        if not content_id or not content_id.strip():
            logger.warning(f"Invalid content_id provided: {content_id}")
            raise HTTPException(status_code=400, detail="Content ID cannot be empty")

        for content in content_data:
            if content.id == content_id:
                # Validate content before returning
                if not content.title or not content.title.strip():
                    logger.error(f"Content {content_id} has invalid title")
                    raise HTTPException(status_code=500, detail="Content has invalid format")

                if content.wordCount < 0:
                    logger.error(f"Content {content_id} has invalid word count: {content.wordCount}")
                    raise HTTPException(status_code=500, detail="Content has invalid word count")

                if content.estimatedReadingTime < 0:
                    logger.error(f"Content {content_id} has invalid reading time: {content.estimatedReadingTime}")
                    raise HTTPException(status_code=500, detail="Content has invalid reading time")

                logger.info(f"Content {content_id} retrieved successfully")
                return content

        logger.warning(f"Content with id {content_id} not found")
        raise HTTPException(status_code=404, detail=f"Content with id {content_id} not found")

    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Unexpected error retrieving content {content_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/module/{module_id}", response_model=List[LearningContent])
async def get_content_by_module(module_id: str):
    """
    Retrieve all content for a specific module
    """
    try:
        # Validate module_id format
        if not module_id or not module_id.strip():
            logger.warning(f"Invalid module_id provided: {module_id}")
            raise HTTPException(status_code=400, detail="Module ID cannot be empty")

        module_content = [content for content in content_data if content.moduleID == module_id]

        if not module_content:
            logger.warning(f"No content found for module {module_id}")
            raise HTTPException(status_code=404, detail=f"No content found for module {module_id}")

        # Validate each content piece
        valid_content = []
        for content in module_content:
            if content.title and content.title.strip():
                if content.wordCount >= 0 and content.estimatedReadingTime >= 0:
                    valid_content.append(content)
                else:
                    logger.warning(f"Content {content.id} has invalid metrics, skipping")
            else:
                logger.warning(f"Content {content.id} has invalid title, skipping")

        if not valid_content:
            logger.error(f"No valid content found for module {module_id} after validation")
            raise HTTPException(status_code=500, detail="No valid content available for this module")

        logger.info(f"Retrieved {len(valid_content)} content pieces for module {module_id}")
        return valid_content

    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Unexpected error retrieving content for module {module_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")