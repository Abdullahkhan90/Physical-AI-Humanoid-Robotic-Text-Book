from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
from dotenv import load_dotenv
from cohere import Client
from qdrant_client import QdrantClient

load_dotenv()

app = FastAPI(title="Physical AI Textbook RAG Chatbot")

# CORS - local + tumhara published frontend link
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "https://abdullahkhan90.github.io",  # tumhara published link
        "https://abdullahkhan90.github.io/Physical-AI-Humanoid-Robotic-Text-Book/"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

cohere_client = Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_ENDPOINT"),
    api_key=os.getenv("QDRANT_API_KEY")
)
collection_name = "textbook_collection"  # ingestion mein jo name use kiya tha

class QueryRequest(BaseModel):
    question: str
    selected_text: str = ""

@app.get("/")
async def root():
    return {"message": "Physical AI Textbook RAG Chatbot Backend is running!"}

@app.get("/api/health")
async def health_check():
    return {"status": "healthy", "message": "Backend is running smoothly!"}

@app.post("/api/query")
async def query(request: QueryRequest):
    return await handle_query(request.question, request.selected_text)

@app.post("/api/query-selected")
async def query_selected(request: QueryRequest):
    return await handle_query(request.question, request.selected_text)

async def handle_query(question: str, selected_text: str = ""):
    try:
        # Embed the question with correct input_type
        embed_response = cohere_client.embed(
            texts=[question],
            model="embed-english-v3.0",
            input_type="search_query"  # yeh zaroori hai new Cohere version mein
        )
        query_vector = embed_response.embeddings[0]

        # Search in Qdrant with payload - check for available methods
        has_query_method = hasattr(qdrant_client, 'query')
        has_search_method = hasattr(qdrant_client, 'search')

        if has_query_method:
            # Use the new 'query' method (Qdrant v1.9.0+)
            search_result = qdrant_client.query(
                collection_name=collection_name,
                query=query_vector,  # In newer versions, it's 'query' not 'query_vector'
                limit=6,
                with_payload=True  # payload include karne ke liye
            )
        elif has_search_method:
            # Use the older 'search' method
            search_result = qdrant_client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=6,
                with_payload=True  # payload include karne ke liye
            )
        else:
            # If neither method is available, return empty results
            print(f"Qdrant client does not have 'query' or 'search' method. Available methods: {[attr for attr in dir(qdrant_client) if not attr.startswith('_')]}")
            search_result = []

        # Build context from retrieved chunks
        context_parts = []
        for hit in search_result:
            if hit.payload and hit.payload.get("text"):
                context_parts.append(hit.payload["text"])
        context = "\n\n".join(context_parts)

        # Add selected text if available
        full_context = selected_text + "\n\n" + context if selected_text else context

        # Generate answer with Cohere - use new chat API instead of deprecated generate
        prompt = f"""Use ONLY the following context to answer the question. Do not add any external knowledge.

Context:
{full_context}

Question: {question}

Answer clearly and accurately:"""

        response = cohere_client.chat(
            model="command-r",
            message=prompt,
            max_tokens=400,
            temperature=0.3
        )

        answer = response.message.content[0].text.strip()

        # Prepare citations
        citations = []
        for hit in search_result:
            if hit.payload:
                citations.append({
                    "file_path": hit.payload.get("file_path", "Unknown file"),
                    "chapter": hit.payload.get("chapter", "Unknown chapter"),
                    "section": hit.payload.get("section", "Unknown section")
                })

        return {
            "answer": answer or "No relevant answer found in the textbook.",
            "citations": citations
        }

    except Exception as e:
        print(f"RAG Error: {e}")
        return {
            "answer": "Sorry, I'm having trouble processing your request right now. Please try again.",
            "citations": []
        }