export interface TextbookModule {
  id: string;
  title: string;
  description: string;
  topics: string[];
  learningObjectives: string[];
  contentPath: string;
  order: number;
  isActive: boolean;
}

export interface LearningContent {
  id: string;
  title: string;
  content: string; // This would be the markdown content
  moduleID: string;
  contentType: 'theory' | 'practical' | 'example' | 'exercise';
  wordCount: number;
  estimatedReadingTime: number;
  requiredCitations: Citation[];
}

export interface StudentProfile {
  id: string;
  preferences: Record<string, any>;
  progress: Record<string, any>;
  personalizationSettings: Record<string, any>;
  preferredLanguage: string;
  createdAt: string; // ISO date string
  lastAccessed: string; // ISO date string
}

export interface RAGKnowledgeBase {
  id: string;
  moduleID: string;
  contentID: string;
  chunkText: string;
  embedding: number[]; // Array of floats
  metadata: Record<string, any>;
}

export interface Citation {
  id: string;
  title: string;
  authors: string[];
  source: string;
  year: number;
  doi?: string;
  url?: string;
  citationType: 'journalArticle' | 'conferencePaper' | 'book' | 'online';
  apaFormatted: string;
}

export interface LearningPath {
  id: string;
  studentID: string;
  moduleSequence: string[];
  personalized: boolean;
  createdAt: string; // ISO date string
  lastModified: string; // ISO date string
}