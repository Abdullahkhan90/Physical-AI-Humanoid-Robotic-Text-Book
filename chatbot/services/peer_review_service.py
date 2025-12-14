from typing import List, Dict, Any
from datetime import datetime
from enum import Enum
import logging


class ReviewStatus(Enum):
    PENDING = "pending"
    IN_REVIEW = "in_review"
    REQUIRES_REVISION = "requires_revision"
    APPROVED = "approved"
    REJECTED = "rejected"


class ReviewResult:
    def __init__(self, reviewer_id: str, content_id: str, status: ReviewStatus, 
                 comments: str = "", timestamp: datetime = None):
        self.reviewer_id = reviewer_id
        self.content_id = content_id
        self.status = status
        self.comments = comments
        self.timestamp = timestamp or datetime.now()
        self.accuracy_score = None  # 0-100 score for content accuracy
        self.clarity_score = None   # 0-100 score for clarity
        self.rigor_score = None     # 0-100 score for academic rigor


class PeerReviewService:
    """
    Service for managing the peer review workflow as required by constitution principles
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.reviews: Dict[str, List[ReviewResult]] = {}  # content_id -> list of reviews
        self.reviewers: Dict[str, Dict[str, Any]] = {}   # reviewer_id -> reviewer info
    
    def add_reviewer(self, reviewer_id: str, name: str, expertise: List[str], 
                     institution: str, is_verified: bool = True):
        """
        Add a reviewer to the system with their expertise
        """
        self.reviewers[reviewer_id] = {
            'name': name,
            'expertise': expertise,
            'institution': institution,
            'is_verified': is_verified,
            'review_count': 0
        }
        self.logger.info(f"Added reviewer {name} with ID {reviewer_id}")
    
    def assign_content_for_review(self, content_id: str, reviewer_ids: List[str]) -> bool:
        """
        Assign content to reviewers for peer review
        """
        if content_id not in self.reviews:
            self.reviews[content_id] = []
        
        for reviewer_id in reviewer_ids:
            if reviewer_id not in self.reviewers:
                self.logger.error(f"Reviewer {reviewer_id} not found in system")
                return False
            
            # Create a new review assignment
            review_result = ReviewResult(
                reviewer_id=reviewer_id,
                content_id=content_id,
                status=ReviewStatus.PENDING
            )
            self.reviews[content_id].append(review_result)
            
            # Update reviewer's stats
            self.reviewers[reviewer_id]['review_count'] += 1
        
        self.logger.info(f"Assigned content {content_id} to {len(reviewer_ids)} reviewers")
        return True
    
    def submit_review(self, reviewer_id: str, content_id: str, status: ReviewStatus,
                     comments: str = "", accuracy_score: int = None,
                     clarity_score: int = None, rigor_score: int = None) -> bool:
        """
        Submit a review for content
        """
        if content_id not in self.reviews:
            self.logger.error(f"No review assignments found for content {content_id}")
            return False
        
        # Find the review assigned to this reviewer
        review_to_update = None
        for review in self.reviews[content_id]:
            if review.reviewer_id == reviewer_id and review.status == ReviewStatus.PENDING:
                review_to_update = review
                break
        
        if not review_to_update:
            self.logger.error(f"No pending review found for {reviewer_id} on content {content_id}")
            return False
        
        # Update the review
        review_to_update.status = status
        review_to_update.comments = comments
        review_to_update.accuracy_score = accuracy_score
        review_to_update.clarity_score = clarity_score
        review_to_update.rigor_score = rigor_score
        
        self.logger.info(f"Review submitted by {reviewer_id} for content {content_id}, status: {status.value}")
        return True
    
    def get_content_review_status(self, content_id: str) -> Dict[str, Any]:
        """
        Get the overall review status for a piece of content
        """
        if content_id not in self.reviews:
            return {
                'status': ReviewStatus.PENDING,
                'reviews': [],
                'approval_percentage': 0
            }
        
        reviews = self.reviews[content_id]
        if not reviews:
            return {
                'status': ReviewStatus.PENDING,
                'reviews': [],
                'approval_percentage': 0
            }
        
        # Calculate status based on all reviews
        approved_count = sum(1 for r in reviews if r.status == ReviewStatus.APPROVED)
        total_reviews = len(reviews)
        
        status = ReviewStatus.PENDING
        if all(r.status == ReviewStatus.APPROVED for r in reviews):
            status = ReviewStatus.APPROVED
        elif any(r.status == ReviewStatus.REJECTED for r in reviews):
            status = ReviewStatus.REJECTED
        elif any(r.status == ReviewStatus.REQUIRES_REVISION for r in reviews):
            status = ReviewStatus.REQUIRES_REVISION
        elif all(r.status in [ReviewStatus.APPROVED, ReviewStatus.IN_REVIEW] for r in reviews):
            status = ReviewStatus.IN_REVIEW
        
        return {
            'status': status,
            'reviews': reviews,
            'approval_percentage': (approved_count / total_reviews) * 100 if total_reviews > 0 else 0
        }
    
    def content_meets_approval_criteria(self, content_id: str, min_approval_percentage: int = 75) -> bool:
        """
        Check if content meets the approval criteria based on peer reviews
        """
        status_info = self.get_content_review_status(content_id)
        
        if status_info['status'] == ReviewStatus.APPROVED:
            # If all reviews are approved, content is approved
            return True
        
        if status_info['approval_percentage'] >= min_approval_percentage:
            # At least the required percentage of reviewers approved
            return True
        
        return False

    def ensure_academic_rigor(self, content_id: str, required_reviews: int = 2) -> bool:
        """
        Ensure content has enough reviews to meet academic rigor standards
        """
        if content_id not in self.reviews:
            return False
        
        submitted_reviews = [r for r in self.reviews[content_id] 
                            if r.status != ReviewStatus.PENDING]
        
        return len(submitted_reviews) >= required_reviews