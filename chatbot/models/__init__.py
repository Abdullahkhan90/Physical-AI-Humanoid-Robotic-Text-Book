from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class TextbookModule(BaseModel):
    id: str
    title: str
    description: str
    topics: List[str]
    learningObjectives: List[str]
    contentPath: str
    order: int
    isActive: bool = True


class LearningContent(BaseModel):
    id: str
    title: str
    content: str
    moduleID: str
    contentType: str
    wordCount: int
    estimatedReadingTime: int
    requiredCitations: List[dict]  # List of Citation objects


class StudentProfile(BaseModel):
    id: str
    preferences: dict
    progress: dict
    personalizationSettings: dict
    preferredLanguage: str = "en"
    createdAt: datetime
    lastAccessed: datetime


class RAGKnowledgeBase(BaseModel):
    id: str
    moduleID: str
    contentID: str
    chunkText: str
    embedding: List[float]
    metadata: dict


class Citation(BaseModel):
    id: str
    title: str
    authors: List[str]
    source: str
    year: int
    doi: Optional[str] = None
    url: Optional[str] = None
    citationType: str
    apaFormatted: str


class LearningPath(BaseModel):
    id: str
    studentID: str
    moduleSequence: List[str]
    personalized: bool
    createdAt: datetime
    lastModified: datetime