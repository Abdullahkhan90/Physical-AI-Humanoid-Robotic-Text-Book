import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# API Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_ENDPOINT = os.getenv("QDRANT_ENDPOINT")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Validation
REQUIRED_ENV_VARS = [
    "COHERE_API_KEY",
    "QDRANT_ENDPOINT",
    "QDRANT_API_KEY"
]

# Only validate required environment variables when running in production
# For testing and development, we allow missing variables but log warnings
import os
missing_vars = [var for var in REQUIRED_ENV_VARS if not os.getenv(var)]
if missing_vars:
    print(f"WARNING: Missing required environment variables: {', '.join(missing_vars)}")
    print("These variables are required for full functionality, but the app will start for testing.")

# Application Configuration
MAX_QUESTION_LENGTH = 1000  # Maximum length for questions
MAX_SELECTED_TEXT_LENGTH = 5000  # Maximum length for selected text
MAX_SOURCE_TEXT_LENGTH = 4000  # Maximum length for retrieved source text