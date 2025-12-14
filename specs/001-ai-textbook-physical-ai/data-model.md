# Data Model for AI Native Textbook on Physical AI & Humanoid Robotics

**Date**: 2025-12-12  
**Feature**: AI Native Textbook on Physical AI & Humanoid Robotics  
**Branch**: `001-ai-textbook-physical-ai`

## Core Entities

### Textbook Module
- **id**: String (unique identifier for the module)
- **title**: String (e.g., "The Robotic Nervous System (ROS 2)")
- **description**: String (brief overview of the module)
- **topics**: Array of strings (specific topics covered in the module)
- **learningObjectives**: Array of strings (what students should learn)
- **contentPath**: String (path to the Markdown content file)
- **order**: Integer (sequence in which modules appear)
- **isActive**: Boolean (whether the module is currently available)

### Learning Content
- **id**: String (unique identifier for the content piece)
- **title**: String (title of the specific content piece)
- **content**: String (Markdown formatted content)
- **moduleID**: String (foreign key to Textbook Module)
- **contentType**: String (e.g., "theory", "practical", "example", "exercise")
- **wordCount**: Integer (number of words in the content)
- **estimatedReadingTime**: Integer (in minutes)
- **requiredCitations**: Array of Citation objects

### Student Profile
- **id**: String (unique identifier for the student)
- **preferences**: Object (learning preferences like language, pace, etc.)
- **progress**: Object (tracking progress through modules)
- **personalizationSettings**: Object (settings for customizing learning path)
- **preferredLanguage**: String (language preference, e.g., "en", "ur")
- **createdAt**: DateTime (when profile was created)
- **lastAccessed**: DateTime (when profile was last accessed)

### RAG Knowledge Base
- **id**: String (unique identifier for the knowledge entry)
- **moduleID**: String (foreign key to Textbook Module)
- **contentID**: String (foreign key to Learning Content)
- **chunkText**: String (portion of content for RAG retrieval)
- **embedding**: Array of Floats (vector embedding of the chunk)
- **metadata**: Object (additional information about the chunk)

### Citation
- **id**: String (unique identifier for the citation)
- **title**: String (title of the cited work)
- **authors**: Array of strings (authors of the cited work)
- **source**: String (journal, conference, etc.)
- **year**: Integer (publication year)
- **doi**: String (digital object identifier, if available)
- **url**: String (URL to the source)
- **citationType**: String (e.g., "journalArticle", "conferencePaper", "book", "online")
- **apaFormatted**: String (APA formatted citation string)

### Learning Path
- **id**: String (unique identifier for the learning path)
- **studentID**: String (foreign key to Student Profile)
- **moduleSequence**: Array of Module IDs (ordered sequence of modules)
- **personalized**: Boolean (whether the path is personalized)
- **createdAt**: DateTime (when path was created)
- **lastModified**: DateTime (when path was last changed)

## Relationships

1. **Textbook Module** 1 → * **Learning Content**: A module contains multiple pieces of learning content
2. **Student Profile** 1 → 1 **Learning Path**: Each student has one learning path
3. **Learning Content** * → * **Citation**: Each content piece can have multiple citations
4. **RAG Knowledge Base** * → 1 **Learning Content**: Multiple knowledge base entries can reference one content piece
5. **RAG Knowledge Base** * → 1 **Textbook Module**: Multiple knowledge base entries can reference one module

## Validation Rules

1. **Textbook Module**:
   - title must not be empty
   - order must be unique within a textbook
   - must have at least one topic

2. **Learning Content**:
   - title must not be empty
   - content must not be empty
   - moduleID must reference an existing module
   - wordCount must be non-negative

3. **Citation**:
   - title must not be empty
   - authors array must have at least one author
   - year must be a valid year (e.g., 1900-2030)
   - citationType must be one of the defined values
   - apaFormatted must follow APA format

4. **Student Profile**:
   - preferredLanguage must be supported by the system (e.g., "en", "ur")

## State Transitions

1. **Textbook Module**:
   - Draft → Active: When module is ready for student access
   - Active → Inactive: When module is temporarily unavailable

2. **Student Profile**:
   - Created → Active: When student starts using the textbook
   - Active → Inactive: When student stops using the textbook for extended period