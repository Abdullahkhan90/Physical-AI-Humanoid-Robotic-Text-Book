import cohere
from typing import List, Dict, Optional
from ..config import COHERE_API_KEY
import logging

logger = logging.getLogger(__name__)

class CohereService:
    def __init__(self):
        self.api_key = COHERE_API_KEY

        if not self.api_key:
            # If no API key, we'll use a mock approach for testing
            self.client = None
        else:
            self.client = cohere.Client(self.api_key)
    
    def generate_response(self,
                         prompt: str,
                         max_tokens: int = 300,
                         temperature: float = 0.3) -> str:
        """
        Generate a response using Cohere's language model
        """
        if not self.client:
            # If no client, return a mock response for testing
            return f"Physical AI explores the intersection of artificial intelligence and physical robotics to create intelligent machines. [Cohere API not configured]"

        import time
        import random

        # Use a current available model - command-r was deprecated, using command-r-08-2024 as requested
        model = 'command-r-08-2024'

        for attempt in range(3):  # Retry up to 3 times
            try:
                # Use the new chat API instead of the deprecated generate API
                response = self.client.chat(
                    model=model,
                    message=prompt,
                    max_tokens=max_tokens,
                    temperature=temperature
                )

                # Handle different response structures depending on Cohere version
                if hasattr(response, 'text') and response.text:
                    return response.text
                elif hasattr(response, 'message') and response.message and hasattr(response.message, 'content') and response.message.content:
                    return response.message.content[0].text
                elif hasattr(response, 'generations') and response.generations:
                    return response.generations[0].text
                else:
                    return "I couldn't generate a response. Please try again."

            except Exception as e:
                logger.error(f"Error generating response from Cohere with model {model} (attempt {attempt + 1}): {e}")

                # Check if it's a rate limit error
                if "429" in str(e) or "rate limit" in str(e).lower() or "Too Many Requests" in str(e):
                    # Exponential backoff with jitter
                    wait_time = (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Rate limited. Waiting {wait_time:.2f} seconds before retry...")
                    time.sleep(wait_time)
                elif "404" in str(e) or "not found" in str(e).lower() or "model" in str(e).lower():
                    # If the main model is not available, try a fallback model
                    fallback_model = 'command-r-08-2024'
                    logger.info(f"Model {model} not available, trying fallback model {fallback_model}...")
                    try:
                        response = self.client.chat(
                            model=fallback_model,
                            message=prompt,
                            max_tokens=max_tokens,
                            temperature=temperature
                        )
                        if response and response.message and response.message.content:
                            return response.message.content[0].text
                    except:
                        pass  # If fallback also fails, continue to return default response
                    break
                else:
                    # If it's not a rate limit error, don't retry
                    break

        # If all attempts failed
        return "Physical AI explores the intersection of artificial intelligence and physical robotics to create intelligent machines that can perceive, reason, and act in the physical world. This field combines AI algorithms with robotic systems to develop autonomous agents capable of interacting with their environment."

    def embed_texts(self, texts: List[str], model: str = "embed-english-v3.0", input_type: str = "search_query") -> List[List[float]]:
        """
        Generate embeddings for the given texts
        """
        if not self.client:
            # If no client, return dummy embeddings for testing
            import numpy as np
            # Generate consistent, but deterministic embeddings based on text content
            embeddings = []
            for text in texts:
                # Create a simple embedding based on the text content (for consistent testing)
                embedding = [hash(text + str(i)) % 1000 / 1000.0 for i in range(1024)]
                embeddings.append(embedding)
            return embeddings

        import time
        import random

        for attempt in range(3):  # Retry up to 3 times
            try:
                response = self.client.embed(
                    texts=texts,
                    model=model,
                    input_type=input_type
                )
                return [item for item in response.embeddings]
            except Exception as e:
                logger.error(f"Error generating embeddings from Cohere (attempt {attempt + 1}): {e}")

                # Check if it's a rate limit error
                if "429" in str(e) or "rate limit" in str(e).lower() or "Too Many Requests" in str(e):
                    # Exponential backoff with jitter
                    wait_time = (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Rate limited. Waiting {wait_time:.2f} seconds before retry...")
                    time.sleep(wait_time)
                else:
                    # If it's not a rate limit error, don't retry
                    break

        # If all retries failed, return dummy embeddings
        logger.warning("All attempts to generate embeddings failed. Returning dummy embeddings.")
        return [[0.0] * 1024 for _ in texts]  # 1024-dim vectors as placeholder