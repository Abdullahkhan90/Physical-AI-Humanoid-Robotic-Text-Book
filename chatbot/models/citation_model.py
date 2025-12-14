from pydantic import BaseModel
from typing import List, Optional


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